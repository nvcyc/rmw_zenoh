// Copyright 2024 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "rmw_subscription_data.hpp"

#include <fastcdr/FastBuffer.h>

#include <algorithm>
#include <cinttypes>
#include <cstring>
#include <iomanip>
#include <limits>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <utility>
#include <variant>

#include "attachment_helpers.hpp"
#include "buffer_backend_loader.hpp"
#include "cdr.hpp"
#include "identifier.hpp"
#include "rmw_context_impl_s.hpp"
#include "message_type_support.hpp"
#include "logging_macros.hpp"
#include "qos.hpp"
#include "liveliness_utils.hpp"

#include "rcpputils/scope_exit.hpp"

#include "rmw/error_handling.h"
#include "rmw/get_topic_endpoint_info.h"
#include "rmw/impl/cpp/macros.hpp"
#include "rosidl_runtime_c/type_hash.h"

#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"

namespace
{
std::string gid_to_hex(const rmw_gid_t & gid)
{
  std::ostringstream out;
  out << std::hex << std::setfill('0');
  for (size_t i = 0; i < RMW_GID_STORAGE_SIZE; ++i) {
    out << std::setw(2) << static_cast<int>(gid.data[i]);
  }
  return out.str();
}
}  // namespace

namespace rmw_zenoh_cpp
{
///=============================================================================
SubscriptionData::Message::Message(
  const zenoh::Bytes & p,
  uint64_t recv_ts,
  AttachmentData && attachment_,
  const rmw_topic_endpoint_info_t * endpoint_info_)
: payload(p), recv_timestamp(recv_ts), attachment(std::move(attachment_)),
  endpoint_info(endpoint_info_)
{
}

///=============================================================================
std::shared_ptr<SubscriptionData> SubscriptionData::make(
  std::shared_ptr<zenoh::Session> session,
  std::shared_ptr<GraphCache> graph_cache,
  const rmw_node_t * const node,
  liveliness::NodeInfo node_info,
  std::size_t node_id,
  std::size_t subscription_id,
  const std::string & topic_name,
  const rosidl_message_type_support_t * type_support,
  const rmw_qos_profile_t * qos_profile,
  const rmw_subscription_options_t & sub_options)
{
  rmw_qos_profile_t adapted_qos_profile = *qos_profile;
  rmw_ret_t ret = QoS::get().best_available_qos(
    node, topic_name.c_str(), &adapted_qos_profile, rmw_get_publishers_info_by_topic);
  if (RMW_RET_OK != ret) {
    return nullptr;
  }

  rcutils_allocator_t * allocator = &node->context->options.allocator;

  const rosidl_type_hash_t * type_hash = type_support->get_type_hash_func(type_support);
  auto callbacks = static_cast<const message_type_support_callbacks_t *>(type_support->data);
  auto message_type_support = std::make_unique<MessageTypeSupport>(callbacks);

  // CREATION-TIME DECISION: Check if message type has Buffer fields
  bool has_buffer_fields = callbacks->has_buffer_fields;
  bool is_buffer_aware = has_buffer_fields;

  std::cerr << "[SubscriptionData::make] Topic: " << topic_name
            << ", has_buffer_fields: " << has_buffer_fields
            << ", is_buffer_aware: " << is_buffer_aware << "\n";

  // Query installed backends if message type has Buffer fields
  std::optional<std::unordered_map<std::string, std::string>> backend_types = std::nullopt;
  std::vector<std::string> my_backend_types;
  if (is_buffer_aware) {
    my_backend_types = rmw_zenoh_cpp::get_installed_backend_types();

    EndpointInfoStorage local_endpoint_info;
    local_endpoint_info.node_name = node_info.name_;
    local_endpoint_info.node_namespace = node_info.ns_;
    local_endpoint_info.topic_type = message_type_support->get_name();
    local_endpoint_info.info.node_name = local_endpoint_info.node_name.c_str();
    local_endpoint_info.info.node_namespace = local_endpoint_info.node_namespace.c_str();
    local_endpoint_info.info.topic_type = local_endpoint_info.topic_type.c_str();
    local_endpoint_info.info.topic_type_hash = *type_hash;
    local_endpoint_info.info.endpoint_type = RMW_ENDPOINT_SUBSCRIPTION;
    std::memset(local_endpoint_info.info.endpoint_gid, 0, RMW_GID_STORAGE_SIZE);
    local_endpoint_info.info.qos_profile = adapted_qos_profile;

    backend_types = rmw_zenoh_cpp::collect_backend_aux_info(
      local_endpoint_info.info, my_backend_types);
    RMW_ZENOH_LOG_DEBUG_NAMED(
      "rmw_zenoh_cpp",
      "Creating Buffer-aware subscription for topic %s with %zu backends",
      topic_name.c_str(), my_backend_types.size());
  }

  // Convert the type hash to a string so that it can be included in the keyexpr.
  char * type_hash_c_str = nullptr;
  rcutils_ret_t stringify_ret = rosidl_stringify_type_hash(
    type_hash,
    *allocator,
    &type_hash_c_str);
  if (RCUTILS_RET_BAD_ALLOC == stringify_ret) {
    // rosidl_stringify_type_hash already set the error
    return nullptr;
  }
  auto free_type_hash_c_str = rcpputils::make_scope_exit(
    [&allocator, &type_hash_c_str]() {
      allocator->deallocate(type_hash_c_str, allocator->state);
    });

  // Everything above succeeded and is setup properly. Now declare a subscriber
  // with Zenoh; after this, callbacks may come in at any time.
  std::size_t domain_id = node_info.domain_id_;
  auto entity = liveliness::Entity::make(
    session->get_zid(),
    std::to_string(node_id),
    std::to_string(subscription_id),
    liveliness::EntityType::Subscription,
    std::move(node_info),
    liveliness::TopicInfo{
      std::move(domain_id),
      topic_name,
      message_type_support->get_name(),
      type_hash_c_str,
      adapted_qos_profile,
      backend_types}  // Include backends only if Buffer message type
  );
  if (entity == nullptr) {
    RMW_ZENOH_LOG_ERROR_NAMED(
      "rmw_zenoh_cpp",
      "Unable to generate keyexpr for liveliness token for the subscription %s.",
      topic_name.c_str());
    return nullptr;
  }

  auto sub_data = std::shared_ptr<SubscriptionData>(
    new SubscriptionData{
      node,
      graph_cache,
      std::move(entity),
      std::move(session),
      type_support->data,
      std::move(message_type_support),
      sub_options,
      is_buffer_aware,
      my_backend_types
    });

  if (is_buffer_aware) {
    auto build_endpoint_info_from_entity =
      [](const liveliness::Entity & entity, rmw_endpoint_type_t endpoint_type)
      -> EndpointInfoStorage
      {
        EndpointInfoStorage storage;
        storage.node_name = entity.node_name();
        storage.node_namespace = entity.node_namespace();
        auto topic_info = entity.topic_info();
        if (topic_info.has_value()) {
          storage.topic_type = topic_info->type_;
          storage.info.qos_profile = topic_info->qos_;
          storage.info.topic_type_hash = rosidl_get_zero_initialized_type_hash();
          (void)rosidl_parse_type_hash_string(
            topic_info->type_hash_.c_str(),
            &storage.info.topic_type_hash);
        }
        storage.info.node_name = storage.node_name.c_str();
        storage.info.node_namespace = storage.node_namespace.c_str();
        storage.info.topic_type = storage.topic_type.c_str();
        storage.info.endpoint_type = endpoint_type;
        auto gid_array = entity.copy_gid();
        std::memcpy(storage.info.endpoint_gid, gid_array.data(), RMW_GID_STORAGE_SIZE);
        return storage;
      };

    sub_data->local_endpoint_info_ =
      build_endpoint_info_from_entity(*sub_data->entity_, RMW_ENDPOINT_SUBSCRIPTION);
  }

  if (!sub_data->init()) {
    // init() already set the error
    return nullptr;
  }

  // Register discovery callback for Buffer-aware subscribers
  if (is_buffer_aware && graph_cache != nullptr) {
    std::weak_ptr<SubscriptionData> weak_sub_data = sub_data;
    graph_cache->register_publisher_discovery_callback(
      topic_name,
      sub_data->gid_hash(),
      [weak_sub_data](const liveliness::Entity & entity) {
        if (auto sd = weak_sub_data.lock()) {
          sd->on_publisher_discovered(entity);
        }
      });

    // Manually add this local subscriber to the graph cache so local publishers can discover it
    // Liveliness events from the same session don't trigger graph updates automatically
    graph_cache->parse_put(sub_data->entity_->liveliness_keyexpr(), false);
    RMW_ZENOH_LOG_INFO_NAMED(
      "rmw_zenoh_cpp",
      "[Subscription] Manually added local subscriber to graph cache for discovery");
  }

  return sub_data;
}

///=============================================================================
SubscriptionData::SubscriptionData(
  const rmw_node_t * rmw_node,
  std::shared_ptr<GraphCache> graph_cache,
  std::shared_ptr<liveliness::Entity> entity,
  std::shared_ptr<zenoh::Session> session,
  const void * type_support_impl,
  std::unique_ptr<MessageTypeSupport> type_support,
  rmw_subscription_options_t sub_options,
  bool is_buffer_aware,
  std::vector<std::string> my_backend_types)
: rmw_node_(rmw_node),
  graph_cache_(std::move(graph_cache)),
  entity_(std::move(entity)),
  sess_(std::move(session)),
  type_support_impl_(type_support_impl),
  type_support_(std::move(type_support)),
  sub_options_(std::move(sub_options)),
  last_known_published_msg_({}),
  wait_set_data_(nullptr),
  is_shutdown_(false),
  initialized_(false),
  is_buffer_aware_(is_buffer_aware),
  my_backend_types_(std::move(my_backend_types))
{
  events_mgr_ = std::make_shared<EventsManager>();
}

///=============================================================================
// We have to use an "init" function here, rather than do this in the constructor, because we use
// enable_shared_from_this, which is not available in constructors.
bool SubscriptionData::init()
{
  zenoh::ZResult result;
  zenoh::KeyExpr sub_ke(entity_->topic_info()->topic_keyexpr_, true, &result);
  if (result != Z_OK) {
    RMW_SET_ERROR_MSG("unable to create zenoh keyexpr.");
    return false;
  }

  rmw_context_impl_t * context_impl = static_cast<rmw_context_impl_t *>(rmw_node_->context->impl);

  sess_ = context_impl->session();

  using AdvancedSubscriberOptions = zenoh::ext::SessionExt::AdvancedSubscriberOptions;
  using RecoveryOptions = AdvancedSubscriberOptions::RecoveryOptions;
  auto adv_sub_opts = AdvancedSubscriberOptions::create_default();

  // By default, this subscription will receive publications from publishers within and outside of
  // the same Zenoh session as this subscription.
  // If ignore_local_publications is true, we restrict this subscription to only receive samples
  // from publishers in remote sessions.
  if (sub_options_.ignore_local_publications) {
    adv_sub_opts.subscriber_options.allowed_origin = ZC_LOCALITY_REMOTE;
  }

  // Instantiate the subscription with suitable options depending on the
  // adapted_qos_profile.
  if (entity_->topic_info()->qos_.durability == RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL) {
    // Allow this subscriber to be detected through liveliness.
    adv_sub_opts.subscriber_detection = true;
    adv_sub_opts.query_timeout_ms = std::numeric_limits<uint64_t>::max();
    // History can only be retransmitted by Publishers that enable caching.
    adv_sub_opts.history = AdvancedSubscriberOptions::HistoryOptions::create_default();
    // Enable detection of late joiner publishers and query for their historical data.
    adv_sub_opts.history->detect_late_publishers = true;
    adv_sub_opts.history->max_samples = entity_->topic_info()->qos_.depth;
    if (entity_->topic_info()->qos_.reliability == RMW_QOS_POLICY_RELIABILITY_RELIABLE) {
      // Activate recovery of lost samples.
      // This requires the Publisher to have sample_miss_detection configured,
      // which is the case for a RELIABLE + TRANSIENT_LOCAL Publisher.
      adv_sub_opts.recovery = AdvancedSubscriberOptions::RecoveryOptions{};
      adv_sub_opts.recovery->last_sample_miss_detection = RecoveryOptions::Heartbeat{};
    }
  }

  // SIMPLE PATH: Create base subscription immediately (non-Buffer messages)
  // COMPLEX PATH: Wait for publisher discovery before creating subscriptions (Buffer messages)
  if (!is_buffer_aware_) {
    std::weak_ptr<SubscriptionData> data_wp = shared_from_this();
    auto on_sample = [data_wp](const zenoh::Sample & sample) {
        auto sub_data = data_wp.lock();
        if (sub_data == nullptr) {
          RMW_ZENOH_LOG_ERROR_NAMED(
            "rmw_zenoh_cpp",
            "SubscriberCallback triggered over %s.",
            std::string(sample.get_keyexpr().as_string_view()).c_str()
          );
          return;
        }
        auto attachment = sample.get_attachment();
        if (!attachment.has_value()) {
          RMW_ZENOH_LOG_ERROR_NAMED(
            "rmw_zenoh_cpp",
            "Unable to obtain attachment for topic '%s'",
            std::string(sample.get_keyexpr().as_string_view()).c_str())
          return;
        }
        auto attachment_value = attachment.value();

        AttachmentData attachment_data(attachment_value);
        sub_data->add_new_message(
          std::make_unique<SubscriptionData::Message>(
            sample.get_payload(),
            get_system_time_in_ns(),
            std::move(attachment_data)),
          std::string(sample.get_keyexpr().as_string_view()));
      };
    sub_ = context_impl->session()->ext().declare_advanced_subscriber(
      sub_ke,
      std::move(on_sample),
      zenoh::closures::none,
      std::move(adv_sub_opts),
      &result);
    if (result != Z_OK) {
      RMW_SET_ERROR_MSG("unable to create zenoh subscription");
      return false;
    }
  } else {
    // Buffer-aware: subscriptions will be created dynamically in on_publisher_discovered
    RMW_ZENOH_LOG_DEBUG_NAMED(
      "rmw_zenoh_cpp",
      "Buffer-aware subscription initialized without base subscription, waiting for publisher discovery");
  }

  // Publish to the graph that a new subscription is in town.
  std::string liveliness_keyexpr = entity_->liveliness_keyexpr();
  token_ = context_impl->session()->liveliness_declare_token(
    zenoh::KeyExpr(liveliness_keyexpr),
    zenoh::Session::LivelinessDeclarationOptions::create_default(),
    &result);
  if (result != Z_OK) {
    RMW_ZENOH_LOG_ERROR_NAMED(
      "rmw_zenoh_cpp",
      "Unable to create liveliness token for the subscription.");
    return false;
  }

  initialized_ = true;

  if (is_buffer_aware_) {
    RMW_ZENOH_LOG_INFO_NAMED(
      "rmw_zenoh_cpp",
      "[Subscription] Initialized buffer-aware subscription, base key: '%s' (endpoints created dynamically)",
      entity_->topic_info()->topic_keyexpr_.c_str());
  } else {
    RMW_ZENOH_LOG_INFO_NAMED(
      "rmw_zenoh_cpp",
      "[Subscription] Initialized simple subscription on key: '%s'",
      entity_->topic_info()->topic_keyexpr_.c_str());
  }

  return true;
}

///=============================================================================
std::size_t SubscriptionData::gid_hash() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return entity_->gid_hash();
}

///=============================================================================
liveliness::TopicInfo SubscriptionData::topic_info() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return entity_->topic_info().value();
}

///=============================================================================
bool SubscriptionData::liveliness_is_valid() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  // The z_check function is now internal in zenoh-1.0.0 so we assume
  // the liveliness token is still initialized as long as this entity has
  // not been shutdown.
  return !is_shutdown_;
}

///=============================================================================
std::shared_ptr<EventsManager> SubscriptionData::events_mgr() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return events_mgr_;
}

///=============================================================================
SubscriptionData::~SubscriptionData()
{
  const rmw_ret_t ret = this->shutdown();
  if (ret != RMW_RET_OK) {
    RMW_ZENOH_LOG_ERROR_NAMED(
      "rmw_zenoh_cpp",
      "Error destructing publisher /%s.",
      entity_->topic_info().value().name_.c_str()
    );
  }
}

///=============================================================================
void SubscriptionData::on_publisher_discovered(const liveliness::Entity & entity)
{
  std::lock_guard<std::mutex> lock(mutex_);

  RMW_ZENOH_LOG_INFO_NAMED(
    "rmw_zenoh_cpp",
    "[Subscription] on_publisher_discovered callback triggered!");

  if (!is_buffer_aware_) {
    return;  // Should not be called for non-Buffer-aware subscriptions
  }

  // Parse publisher backend list from liveliness key
  const auto & topic_info = entity.topic_info();
  if (!topic_info.has_value() || !topic_info->backend_aux_info_.has_value()) {
    RMW_ZENOH_LOG_WARN_NAMED(
      "rmw_zenoh_cpp",
      "Discovered publisher without backend info on Buffer topic");
    return;
  }

  std::vector<std::string> pub_backends;
  pub_backends.reserve(topic_info->backend_aux_info_->size());
  for (const auto & pair : topic_info->backend_aux_info_.value()) {
    pub_backends.push_back(pair.first);
  }

  // Check backend compatibility
  if (!rmw_zenoh_cpp::backends_compatible(my_backend_types_, pub_backends)) {
    RMW_ZENOH_LOG_DEBUG_NAMED(
      "rmw_zenoh_cpp",
      "Discovered publisher with incompatible backends, skipping");
    return;
  }

  rmw_gid_t pub_gid = rmw_zenoh_cpp::entity_gid_to_rmw_gid(entity,
      rmw_zenoh_cpp::rmw_zenoh_identifier);

  for (const auto & existing : discovered_publishers_) {
    if (memcmp(existing.gid.data, pub_gid.data, RMW_GID_STORAGE_SIZE) == 0) {
      return;
    }
  }

  auto build_endpoint_info_from_entity =
    [](const liveliness::Entity & entity, rmw_endpoint_type_t endpoint_type)
    -> EndpointInfoStorage
    {
      EndpointInfoStorage storage;
      storage.node_name = entity.node_name();
      storage.node_namespace = entity.node_namespace();
      auto topic_info = entity.topic_info();
      if (topic_info.has_value()) {
        storage.topic_type = topic_info->type_;
        storage.info.qos_profile = topic_info->qos_;
        storage.info.topic_type_hash = rosidl_get_zero_initialized_type_hash();
        (void)rosidl_parse_type_hash_string(
          topic_info->type_hash_.c_str(),
          &storage.info.topic_type_hash);
      }
      storage.info.node_name = storage.node_name.c_str();
      storage.info.node_namespace = storage.node_namespace.c_str();
      storage.info.topic_type = storage.topic_type.c_str();
      storage.info.endpoint_type = endpoint_type;
      auto gid_array = entity.copy_gid();
      std::memcpy(storage.info.endpoint_gid, gid_array.data(), RMW_GID_STORAGE_SIZE);
      return storage;
    };

  auto pub_endpoint_info = build_endpoint_info_from_entity(entity, RMW_ENDPOINT_PUBLISHER);

  std::vector<rmw_topic_endpoint_info_t> existing_endpoints;
  existing_endpoints.reserve(1 + discovered_publishers_.size());
  existing_endpoints.push_back(local_endpoint_info_.info);
  for (const auto & existing : discovered_publishers_) {
    existing_endpoints.push_back(existing.endpoint_info.info);
  }

  std::unordered_map<std::string, std::vector<std::set<uint32_t>>> backend_groups;
  auto backend_compat = rmw_zenoh_cpp::evaluate_backend_compatibility(
    pub_endpoint_info.info, existing_endpoints, backend_groups);

  rmw_gid_t local_gid = {};
  auto local_gid_array = entity_->copy_gid();
  std::memcpy(local_gid.data, local_gid_array.data(), RMW_GID_STORAGE_SIZE);

  std::string full_key = entity_->topic_info()->topic_keyexpr_ + "/" +
    entity.zid() + "/" + gid_to_hex(local_gid);

  RMW_ZENOH_LOG_INFO_NAMED(
    "rmw_zenoh_cpp",
    "[Subscription] Discovered publisher! key='%s'", full_key.c_str());

  // Create subscription for this key if not already exists
  if (sub_endpoints_.find(full_key) == sub_endpoints_.end()) {
    create_subscription_for_key(full_key, pub_endpoint_info);
  }

  // Track publisher
  PublisherInfo pub_info;
  pub_info.gid = pub_gid;
  pub_info.endpoint_key = full_key;
  pub_info.endpoint_info = std::move(pub_endpoint_info);
  pub_info.backend_aux_info = topic_info->backend_aux_info_.value();
  pub_info.backend_compat = std::move(backend_compat);
  pub_info.backend_groups = std::move(backend_groups);
  discovered_publishers_.push_back(std::move(pub_info));
}

///=============================================================================
void SubscriptionData::create_subscription_for_key(
  const std::string & key,
  const EndpointInfoStorage & publisher_info)
{
  zenoh::ZResult result;
  zenoh::KeyExpr sub_ke(key, true, &result);
  if (result != Z_OK) {
    RMW_ZENOH_LOG_ERROR_NAMED(
      "rmw_zenoh_cpp",
      "Unable to create zenoh keyexpr for key: %s", key.c_str());
    return;
  }

  rmw_context_impl_t * context_impl = static_cast<rmw_context_impl_t *>(rmw_node_->context->impl);

  using AdvancedSubscriberOptions = zenoh::ext::SessionExt::AdvancedSubscriberOptions;
  using RecoveryOptions = AdvancedSubscriberOptions::RecoveryOptions;
  auto adv_sub_opts = AdvancedSubscriberOptions::create_default();

  if (sub_options_.ignore_local_publications) {
    adv_sub_opts.subscriber_options.allowed_origin = ZC_LOCALITY_REMOTE;
  }

  if (entity_->topic_info()->qos_.durability == RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL) {
    adv_sub_opts.subscriber_detection = true;
    adv_sub_opts.query_timeout_ms = std::numeric_limits<uint64_t>::max();
    adv_sub_opts.history = AdvancedSubscriberOptions::HistoryOptions::create_default();
    adv_sub_opts.history->detect_late_publishers = true;
    adv_sub_opts.history->max_samples = entity_->topic_info()->qos_.depth;
    if (entity_->topic_info()->qos_.reliability == RMW_QOS_POLICY_RELIABILITY_RELIABLE) {
      adv_sub_opts.recovery = AdvancedSubscriberOptions::RecoveryOptions{};
      adv_sub_opts.recovery->last_sample_miss_detection = RecoveryOptions::Heartbeat{};
    }
  }

  auto endpoint = std::make_shared<SubscriptionEndpoint>();
  endpoint->key = key;
  endpoint->publisher_info = publisher_info;
  const rmw_topic_endpoint_info_t * endpoint_info_ptr = &endpoint->publisher_info.info;

  std::weak_ptr<SubscriptionData> data_wp = shared_from_this();
  auto on_sample = [data_wp, endpoint_info_ptr](const zenoh::Sample & sample) {
      auto sub_data = data_wp.lock();
      if (sub_data == nullptr) {
        return;
      }

      auto attachment = sample.get_attachment();
      if (!attachment.has_value()) {
        RMW_ZENOH_LOG_ERROR_NAMED(
          "rmw_zenoh_cpp",
          "Unable to obtain attachment for topic '%s'",
          std::string(sample.get_keyexpr().as_string_view()).c_str());
        return;
      }

      AttachmentData attachment_data(attachment.value());

      sub_data->add_new_message(
        std::make_unique<SubscriptionData::Message>(
          sample.get_payload(),
          get_system_time_in_ns(),
          std::move(attachment_data),
          endpoint_info_ptr),
        std::string(sample.get_keyexpr().as_string_view()));
    };

  auto sub = context_impl->session()->ext().declare_advanced_subscriber(
    sub_ke,
    std::move(on_sample),
    zenoh::closures::none,
    std::move(adv_sub_opts),
    &result);

  if (result != Z_OK) {
    RMW_ZENOH_LOG_ERROR_NAMED(
      "rmw_zenoh_cpp",
      "Unable to create zenoh subscription for key: %s", key.c_str());
    return;
  }

  endpoint->sub = std::optional<zenoh::ext::AdvancedSubscriber<void>>(std::move(sub));
  sub_endpoints_[key] = endpoint;

  RMW_ZENOH_LOG_INFO_NAMED(
    "rmw_zenoh_cpp",
    "[Subscription] Created buffer-aware subscription for key: '%s'",
    key.c_str());
}

///=============================================================================
rmw_ret_t SubscriptionData::shutdown()
{
  rmw_ret_t ret = RMW_RET_OK;
  std::lock_guard<std::mutex> lock(mutex_);
  if (is_shutdown_ || !initialized_) {
    return ret;
  }

  // Remove any event callbacks registered to this subscription.
  graph_cache_->remove_qos_event_callbacks(entity_->gid_hash());

  // Unregister discovery callbacks if Buffer-aware
  if (is_buffer_aware_) {
    graph_cache_->unregister_discovery_callbacks(entity_->gid_hash());
  }

  // Unregister this subscription from the ROS graph.
  zenoh::ZResult result;
  std::move(token_).value().undeclare(&result);
  if (result != Z_OK) {
    RMW_ZENOH_LOG_ERROR_NAMED(
      "rmw_zenoh_cpp",
      "Unable to undeclare the liveliness token for topic '%s'",
      entity_->topic_info().value().name_.c_str());
    return RMW_RET_ERROR;
  }

  // Undeclare all dynamic subscriptions for Buffer-aware subscriptions
  for (auto & [key, endpoint] : sub_endpoints_) {
    if (endpoint->sub.has_value()) {
      std::move(endpoint->sub.value()).undeclare(&result);
      if (result != Z_OK) {
        RMW_ZENOH_LOG_ERROR_NAMED(
          "rmw_zenoh_cpp",
          "Unable to undeclare subscription for key '%s'",
          key.c_str());
        ret = RMW_RET_ERROR;
      }
    }
  }
  sub_endpoints_.clear();

  if (sub_.has_value()) {
    std::move(sub_.value()).undeclare(&result);
    if (result != Z_OK) {
      RMW_ZENOH_LOG_ERROR_NAMED(
        "rmw_zenoh_cpp",
        "Unable to undeclare the subscriber for topic '%s'",
        entity_->topic_info().value().name_.c_str());
      return RMW_RET_ERROR;
    }
  }

  sess_.reset();
  is_shutdown_ = true;
  initialized_ = false;
  return ret;
}

///=============================================================================
bool SubscriptionData::is_shutdown() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return is_shutdown_;
}

///=============================================================================
bool SubscriptionData::queue_has_data_and_attach_condition_if_not(
  rmw_wait_set_data_t * wait_set_data)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!message_queue_.empty()) {
    return true;
  }

  wait_set_data_ = wait_set_data;

  return false;
}

///=============================================================================
bool SubscriptionData::detach_condition_and_queue_is_empty()
{
  std::lock_guard<std::mutex> lock(mutex_);
  wait_set_data_ = nullptr;

  return message_queue_.empty();
}

///=============================================================================
rmw_ret_t SubscriptionData::take_one_message(
  void * ros_message,
  rmw_message_info_t * message_info,
  bool * taken)
{
  *taken = false;

  std::lock_guard<std::mutex> lock(mutex_);
  if (is_shutdown_ || message_queue_.empty()) {
    // This tells rcl that the check for a new message was done, but no messages have come in yet.
    return RMW_RET_OK;
  }
  std::unique_ptr<Message> msg_data = std::move(message_queue_.front());
  message_queue_.pop_front();

  const Payload & payload_data = msg_data->payload;

  if (payload_data.empty()) {
    RMW_ZENOH_LOG_DEBUG_NAMED(
      "rmw_zenoh_cpp",
      "SubscriptionData not able to get slice data");
    return RMW_RET_ERROR;
  }
  RMW_ZENOH_LOG_INFO_NAMED(
    "rmw_zenoh_cpp",
    "[Subscription] Preparing to deserialize message, payload size: %zu bytes",
    payload_data.size());

  // Object that manages the raw buffer
  // FastCDR needs extra space for internal operations during deserialization
  // Allocate a larger buffer and copy the payload data
  size_t buffer_size = payload_data.size() * 4 + 65536;  // 4x + 64KB safety margin (very conservative)
  rcutils_allocator_t * allocator = &rmw_node_->context->options.allocator;
  void * buffer_data = allocator->allocate(buffer_size, allocator->state);
  if (buffer_data == nullptr) {
    RMW_SET_ERROR_MSG("failed to allocate deserialization buffer");
    return RMW_RET_ERROR;
  }
  auto cleanup_buffer = rcpputils::make_scope_exit(
    [allocator, buffer_data]() {
      allocator->deallocate(buffer_data, allocator->state);
    });

  // Copy payload data to the larger buffer
  std::cerr << "[take_one_message] About to copy " << payload_data.size() <<
    " bytes to buffer (allocated: " << buffer_size << " bytes)\n";
  std::memcpy(buffer_data, payload_data.data(), payload_data.size());
  std::cerr << "[take_one_message] Memory copy complete\n";

  // FastCDR needs to know the actual data size, not the buffer size
  std::cerr << "[take_one_message] Creating FastBuffer with payload_size=" <<
    payload_data.size() << "\n";
  eprosima::fastcdr::FastBuffer fastbuffer(
    reinterpret_cast<char *>(buffer_data),
    payload_data.size());  // Use actual payload size, not allocated buffer size

  std::cerr << "[take_one_message] Creating Cdr deserializer\n";
  // Object that deserializes the data
  rmw_zenoh_cpp::Cdr deser(fastbuffer);

  std::cerr << "[take_one_message] FastBuffer created with buffer_size=" << buffer_size
            << ", payload_size=" << payload_data.size()
            << ", is_buffer_aware_=" << is_buffer_aware_ << "\n";

  RMW_ZENOH_LOG_INFO_NAMED(
    "rmw_zenoh_cpp",
    "[Subscription] FastBuffer created, starting deserialization...");

  // Use endpoint-aware deserialization for Buffer-aware subscriptions
  bool deserialize_success = false;

  try {
    if (is_buffer_aware_) {
      RMW_ZENOH_LOG_INFO_NAMED(
        "rmw_zenoh_cpp",
        "[Subscription] Using endpoint-aware deserialization");

      std::cerr << "[take_one_message] Calling deserialize_ros_message_with_endpoint\n";
      std::cerr << "[take_one_message] CDR state before deserialize - buffer_size=" <<
        buffer_size << ", payload_size=" << payload_data.size() << "\n";

      const rmw_topic_endpoint_info_t empty_endpoint_info =
        rmw_get_zero_initialized_topic_endpoint_info();
      const rmw_topic_endpoint_info_t * endpoint_info =
        msg_data->endpoint_info != nullptr ? msg_data->endpoint_info : &empty_endpoint_info;

      // Use type_support_->deserialize_ros_message_with_endpoint() which handles encapsulation reading
      deserialize_success = type_support_->deserialize_ros_message_with_endpoint(
        deser.get_cdr(),
        ros_message,
        type_support_impl_,
        *endpoint_info);

      std::cerr << "[take_one_message] deserialize_ros_message_with_endpoint returned: " <<
        deserialize_success << "\n";
    } else {
      // Simple path: standard deserialization
      deserialize_success = type_support_->deserialize_ros_message(
        deser.get_cdr(),
        ros_message,
        type_support_impl_);
    }
  } catch (const std::exception & e) {
    std::cerr << "[take_one_message] EXCEPTION CAUGHT: " << e.what() << "\n";
    RMW_ZENOH_LOG_ERROR_NAMED(
      "rmw_zenoh_cpp",
      "[Subscription] EXCEPTION during deserialization: %s", e.what());
    RMW_SET_ERROR_MSG_WITH_FORMAT_STRING("Deserialization exception: %s", e.what());
    return RMW_RET_ERROR;
  }

  if (!deserialize_success) {
    RMW_SET_ERROR_MSG("could not deserialize ROS message");
    return RMW_RET_ERROR;
  }

  RMW_ZENOH_LOG_INFO_NAMED(
    "rmw_zenoh_cpp",
    "[Subscription] Deserialization completed successfully");

  if (message_info != nullptr) {
    message_info->source_timestamp = msg_data->attachment.source_timestamp();
    message_info->received_timestamp = msg_data->recv_timestamp;
    message_info->publication_sequence_number = msg_data->attachment.sequence_number();
    // TODO(clalancette): fill in reception_sequence_number
    message_info->reception_sequence_number = 0;
    message_info->publisher_gid.implementation_identifier = rmw_zenoh_cpp::rmw_zenoh_identifier;
    memcpy(
      message_info->publisher_gid.data,
      msg_data->attachment.copy_gid().data(),
      RMW_GID_STORAGE_SIZE);
    message_info->from_intra_process = false;
  }
  *taken = true;

  return RMW_RET_OK;
}

///=============================================================================
rmw_ret_t SubscriptionData::take_serialized_message(
  rmw_serialized_message_t * serialized_message,
  bool * taken,
  rmw_message_info_t * message_info)
{
  *taken = false;

  std::lock_guard<std::mutex> lock(mutex_);
  if (is_shutdown_ || message_queue_.empty()) {
    // This tells rcl that the check for a new message was done, but no messages have come in yet.
    return RMW_RET_OK;
  }
  std::unique_ptr<Message> msg_data = std::move(message_queue_.front());
  message_queue_.pop_front();

  const Payload & payload_data = msg_data->payload;

  if (payload_data.empty()) {
    RMW_ZENOH_LOG_DEBUG_NAMED(
      "rmw_zenoh_cpp",
      "SubscriptionData not able to get slice data");
    return RMW_RET_ERROR;
  }
  if (serialized_message->buffer_capacity < payload_data.size()) {
    rmw_ret_t ret =
      rmw_serialized_message_resize(serialized_message, payload_data.size());
    if (ret != RMW_RET_OK) {
      return ret;  // Error message already set
    }
  }
  serialized_message->buffer_length = payload_data.size();
  memcpy(
    serialized_message->buffer,
    payload_data.data(),
    payload_data.size());

  *taken = true;

  if (message_info != nullptr) {
    message_info->source_timestamp = msg_data->attachment.source_timestamp();
    message_info->received_timestamp = msg_data->recv_timestamp;
    message_info->publication_sequence_number = msg_data->attachment.sequence_number();
    // TODO(clalancette): fill in reception_sequence_number
    message_info->reception_sequence_number = 0;
    message_info->publisher_gid.implementation_identifier = rmw_zenoh_cpp::rmw_zenoh_identifier;
    memcpy(
      message_info->publisher_gid.data,
      msg_data->attachment.copy_gid().data(),
      RMW_GID_STORAGE_SIZE);
    message_info->from_intra_process = false;
  }

  return RMW_RET_OK;
}

///=============================================================================
void SubscriptionData::add_new_message(
  std::unique_ptr<SubscriptionData::Message> msg,
  const std::string & topic_name)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (is_shutdown_) {
    return;
  }
  const rmw_qos_profile_t adapted_qos_profile = entity_->topic_info().value().qos_;
  if (adapted_qos_profile.history != RMW_QOS_POLICY_HISTORY_KEEP_ALL &&
    message_queue_.size() >= adapted_qos_profile.depth)
  {
    // Log warning if message is discarded due to hitting the queue depth
    RMW_ZENOH_LOG_DEBUG_NAMED(
      "rmw_zenoh_cpp",
      "Message queue depth of %ld reached, discarding oldest message "
      "for subscription for %s",
      adapted_qos_profile.depth,
      topic_name.c_str());

    // If the adapted_qos_profile.depth is 0, the std::move command below will result
    // in UB and the z_drop will segfault. We explicitly set the depth to a minimum of 1
    // in rmw_create_subscription() but to be safe, we only attempt to discard from the
    // queue if it is non-empty.
    if (!message_queue_.empty()) {
      std::unique_ptr<Message> old = std::move(message_queue_.front());
      message_queue_.pop_front();
    }
  }

  // Check for messages lost if the new sequence number is not monotonically increasing.
  const size_t gid_hash = hash_gid(msg->attachment.copy_gid());
  auto last_known_pub_it = last_known_published_msg_.find(gid_hash);
  if (last_known_pub_it != last_known_published_msg_.end()) {
    const int64_t seq_increment = std::abs(
      msg->attachment.sequence_number() -
      last_known_pub_it->second);
    if (seq_increment > 1) {
      int32_t num_msg_lost =
        static_cast<int32_t>(std::clamp(
          seq_increment - 1,
          static_cast<int64_t>(std::numeric_limits<int32_t>::min()),
          static_cast<int64_t>(std::numeric_limits<int32_t>::max())));
      events_mgr_->update_event_status(
        ZENOH_EVENT_MESSAGE_LOST,
        std::move(num_msg_lost));
    }
  }
  // Always update the last known sequence number for the publisher.
  last_known_published_msg_[gid_hash] = msg->attachment.sequence_number();

  message_queue_.emplace_back(std::move(msg));

  // Since we added new data, trigger user callback and guard condition if they are available
  data_callback_mgr_.trigger_callback();
  if (wait_set_data_ != nullptr) {
    std::lock_guard<std::mutex> wait_set_lock(wait_set_data_->condition_mutex);
    wait_set_data_->triggered = true;
    wait_set_data_->condition_variable.notify_one();
  }
}

//==============================================================================
void SubscriptionData::set_on_new_message_callback(
  rmw_event_callback_t callback,
  const void * user_data)
{
  std::lock_guard<std::mutex> lock(mutex_);
  data_callback_mgr_.set_callback(user_data, std::move(callback));
}

//==============================================================================
std::shared_ptr<GraphCache> SubscriptionData::graph_cache() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return graph_cache_;
}

}  // namespace rmw_zenoh_cpp
