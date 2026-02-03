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

#include "rmw_publisher_data.hpp"

#include <fastcdr/FastBuffer.h>

#include <array>
#include <cinttypes>
#include <cstring>
#include <iomanip>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <string>
#include <utility>
#include <variant>
#include <vector>
#include <cstdint>

#include "buffer_backend_loader.hpp"
#include "cdr.hpp"
#include "identifier.hpp"
#include "rmw_context_impl_s.hpp"
#include "message_type_support.hpp"
#include "logging_macros.hpp"
#include "qos.hpp"
#include "zenoh_utils.hpp"

#include "rcpputils/scope_exit.hpp"

#include "rmw/error_handling.h"
#include "rmw/get_topic_endpoint_info.h"
#include "rmw/impl/cpp/macros.hpp"
#include "rosidl_runtime_c/type_hash.h"

#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"

#include "tracetools/tracetools.h"

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

std::string gid_array_to_hex(const std::array<uint8_t, RMW_GID_STORAGE_SIZE> & gid_array)
{
  std::ostringstream out;
  out << std::hex << std::setfill('0');
  for (size_t i = 0; i < gid_array.size(); ++i) {
    out << std::setw(2) << static_cast<int>(gid_array[i]);
  }
  return out.str();
}

const char * entity_type_to_string(rmw_zenoh_cpp::liveliness::EntityType type)
{
  switch (type) {
    case rmw_zenoh_cpp::liveliness::EntityType::Node:
      return "Node";
    case rmw_zenoh_cpp::liveliness::EntityType::Publisher:
      return "Publisher";
    case rmw_zenoh_cpp::liveliness::EntityType::Subscription:
      return "Subscription";
    case rmw_zenoh_cpp::liveliness::EntityType::Service:
      return "Service";
    case rmw_zenoh_cpp::liveliness::EntityType::Client:
      return "Client";
    default:
      return "Unknown";
  }
}
}  // namespace

namespace rmw_zenoh_cpp
{
// Period (ms) of heartbeats sent for detection of lost samples
// by a RELIABLE + TRANSIENT_LOCAL Publisher
#define SAMPLE_MISS_DETECTION_HEARTBEAT_PERIOD 500

///=============================================================================
std::shared_ptr<PublisherData> PublisherData::make(
  std::shared_ptr<zenoh::Session> session,
  const rmw_publisher_t * const rmw_publisher,
  const rmw_node_t * const node,
  liveliness::NodeInfo node_info,
  std::size_t node_id,
  std::size_t publisher_id,
  const std::string & topic_name,
  const rosidl_message_type_support_t * type_support,
  const rmw_qos_profile_t * qos_profile)
{
  rmw_qos_profile_t adapted_qos_profile = *qos_profile;
  rmw_ret_t ret = QoS::get().best_available_qos(
    node, topic_name.c_str(), &adapted_qos_profile, rmw_get_subscriptions_info_by_topic);
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

  std::cerr << "[PublisherData::make] Topic: " << topic_name
            << ", has_buffer_fields: " << has_buffer_fields
            << ", is_buffer_aware: " << is_buffer_aware << "\n";

  // Query installed backends if message type has Buffer fields
  std::unordered_map<std::string, std::string> backend_aux_info;
  std::vector<std::string> my_backend_types;
  if (is_buffer_aware) {
    my_backend_types = rmw_zenoh_cpp::get_installed_backend_types();
    backend_aux_info = rmw_zenoh_cpp::collect_backend_aux_info();
    std::cerr << "[PublisherData::make] Found " << my_backend_types.size() << " backends\n";
    RMW_ZENOH_LOG_DEBUG_NAMED(
      "rmw_zenoh_cpp",
      "Creating Buffer-aware publisher for topic %s with %zu backends",
      topic_name.c_str(), my_backend_types.size());
  }

  // Convert the type hash to a string so that it can be included in
  // the keyexpr.
  char * type_hash_c_str = nullptr;
  rcutils_ret_t stringify_ret = rosidl_stringify_type_hash(
    type_hash,
    *allocator,
    &type_hash_c_str);
  if (RCUTILS_RET_BAD_ALLOC == stringify_ret) {
    // rosidl_stringify_type_hash already set the error
    return nullptr;
  }
  auto always_free_type_hash_c_str = rcpputils::make_scope_exit(
    [&allocator, &type_hash_c_str]() {
      allocator->deallocate(type_hash_c_str, allocator->state);
    });

  std::size_t domain_id = node_info.domain_id_;
  auto entity = liveliness::Entity::make(
    session->get_zid(),
    std::to_string(node_id),
    std::to_string(publisher_id),
    liveliness::EntityType::Publisher,
    std::move(node_info),
    liveliness::TopicInfo{
      std::move(domain_id),
      topic_name,
      message_type_support->get_name(),
      type_hash_c_str,
      adapted_qos_profile,
      is_buffer_aware ? std::make_optional(backend_aux_info) : std::nullopt}
  );
  if (entity == nullptr) {
    RMW_ZENOH_LOG_ERROR_NAMED(
      "rmw_zenoh_cpp",
      "Unable to generate keyexpr for liveliness token for the publisher %s.",
      topic_name.c_str());
    return nullptr;
  }

  using AdvancedPublisherOptions = zenoh::ext::SessionExt::AdvancedPublisherOptions;
  using SampleMissDetectionOptions = AdvancedPublisherOptions::SampleMissDetectionOptions;
  auto adv_pub_opts = AdvancedPublisherOptions::create_default();

  if (adapted_qos_profile.durability == RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL) {
    // Allow this publisher to be detected through liveliness.
    adv_pub_opts.publisher_detection = true;
    adv_pub_opts.cache = AdvancedPublisherOptions::CacheOptions::create_default();
    adv_pub_opts.cache->max_samples = adapted_qos_profile.depth;
    if (adapted_qos_profile.reliability == RMW_QOS_POLICY_RELIABILITY_RELIABLE) {
      // If RELIABLE + TRANSIENT_LOCAL activate sample miss detection for subscriber
      // to detect missed samples and retrieve those from the Publisher cache.
      // HeartbeatSporadic is used to prevent excessive background traffic
      adv_pub_opts.sample_miss_detection = SampleMissDetectionOptions{};
      adv_pub_opts.sample_miss_detection->heartbeat =
        SampleMissDetectionOptions::HeartbeatSporadic{
        SAMPLE_MISS_DETECTION_HEARTBEAT_PERIOD};
    }
  }

  zenoh::KeyExpr pub_ke(entity->topic_info()->topic_keyexpr_);
  // Set congestion_control to BLOCK if appropriate.
  auto pub_opts = zenoh::Session::PublisherOptions::create_default();
  pub_opts.congestion_control = Z_CONGESTION_CONTROL_DROP;
  if (adapted_qos_profile.reliability == RMW_QOS_POLICY_RELIABILITY_RELIABLE) {
    pub_opts.reliability = Z_RELIABILITY_RELIABLE;
    if (adapted_qos_profile.history == RMW_QOS_POLICY_HISTORY_KEEP_ALL) {
      pub_opts.congestion_control = Z_CONGESTION_CONTROL_BLOCK;
    }
  } else {
    pub_opts.reliability = Z_RELIABILITY_BEST_EFFORT;
  }
  adv_pub_opts.publisher_options = pub_opts;

  zenoh::ZResult result;
  auto adv_pub = session->ext().declare_advanced_publisher(
    pub_ke, std::move(adv_pub_opts), &result);
  if (result != Z_OK) {
    RMW_SET_ERROR_MSG("unable to create zenoh publisher cache");
    return nullptr;
  }

  if (result != Z_OK) {
    RMW_SET_ERROR_MSG("Unable to create Zenoh publisher.");
    return nullptr;
  }

  std::string liveliness_keyexpr = entity->liveliness_keyexpr();
  auto token = session->liveliness_declare_token(
    zenoh::KeyExpr(liveliness_keyexpr),
    zenoh::Session::LivelinessDeclarationOptions::create_default(),
    &result);
  if (result != Z_OK) {
    RMW_ZENOH_LOG_ERROR_NAMED(
      "rmw_zenoh_cpp",
      "Unable to create liveliness token for the publisher.");
    return nullptr;
  }

  auto pub_data = std::shared_ptr<PublisherData>(
    new PublisherData{
      rmw_publisher,
      node,
      std::move(entity),
      std::move(session),
      std::move(adv_pub),
      std::move(token),
      type_support->data,
      std::move(message_type_support),
      is_buffer_aware,
      backend_aux_info
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

    pub_data->local_endpoint_info_ =
      build_endpoint_info_from_entity(*pub_data->entity_, RMW_ENDPOINT_PUBLISHER);

    // Inform backends AFTER the GID is properly set
    rmw_zenoh_cpp::inform_backends_on_creating_endpoint(pub_data->local_endpoint_info_.info);
  }

  rmw_context_impl_t * context_impl = static_cast<rmw_context_impl_t *>(node->context->impl);

  // Register discovery callback for Buffer-aware publishers
  if (is_buffer_aware) {
    pub_data->graph_cache_ = context_impl->graph_cache();
    if (pub_data->graph_cache_ != nullptr) {
      std::weak_ptr<PublisherData> weak_pub_data = pub_data;
      pub_data->graph_cache_->register_subscriber_discovery_callback(
        topic_name,
        pub_data->gid_hash(),
        [weak_pub_data](const liveliness::Entity & entity) {
          if (auto pd = weak_pub_data.lock()) {
            pd->on_subscriber_discovered(entity);
          }
        });

      // Manually add this local publisher to the graph cache so local subscribers can discover it
      // Liveliness events from the same session don't trigger graph updates automatically
      pub_data->graph_cache_->parse_put(pub_data->entity_->liveliness_keyexpr(), false);
    }
  }

  return pub_data;
}

///=============================================================================
PublisherData::PublisherData(
  const rmw_publisher_t * const rmw_publisher,
  const rmw_node_t * rmw_node,
  std::shared_ptr<liveliness::Entity> entity,
  std::shared_ptr<zenoh::Session> sess,
  zenoh::ext::AdvancedPublisher pub,
  zenoh::LivelinessToken token,
  const void * type_support_impl,
  std::unique_ptr<MessageTypeSupport> type_support,
  bool is_buffer_aware,
  std::unordered_map<std::string, std::string> backend_aux_info)
: rmw_publisher_(rmw_publisher),
  rmw_node_(rmw_node),
  entity_(std::move(entity)),
  sess_(std::move(sess)),
  pub_(std::move(pub)),
  token_(std::move(token)),
  type_support_impl_(type_support_impl),
  type_support_(std::move(type_support)),
  sequence_number_(1),
  is_shutdown_(false),
  is_buffer_aware_(is_buffer_aware),
  backend_aux_info_(std::move(backend_aux_info))
{
  events_mgr_ = std::make_shared<EventsManager>();

  // For simple publishers, create a single base endpoint
  if (!is_buffer_aware_) {
    auto base_endpoint = std::make_shared<PublisherEndpoint>();
    base_endpoint->key = entity_->topic_info()->topic_keyexpr_;
    base_endpoint->pub = std::optional<zenoh::ext::AdvancedPublisher>(std::move(pub_));
    endpoints_[base_endpoint->key] = base_endpoint;
  }
  // For buffer-aware publishers, endpoints are created dynamically on subscriber discovery
}

///=============================================================================
// Helper function for buffer-aware publishing
rmw_ret_t PublisherData::publish_buffer_aware(
  const void * ros_message,
  ShmContext * shm)
{
  (void)shm;  // SHM not currently used for buffer-aware publishing

  // For buffer-aware publishers, route to different endpoints based on discovered subscribers
  if (discovered_subscribers_.empty()) {
    // No subscribers yet, skip publish
    return RMW_RET_OK;
  }

  // Clear message caches
  for (auto & [key, ep] : endpoints_) {
    ep->cached_message.reset();
  }

  // Serialize and publish to each endpoint
  rmw_ret_t ret = RMW_RET_OK;
  size_t iteration = 0;
  for (auto & sub : discovered_subscribers_) {
    iteration++;
    RMW_ZENOH_LOG_INFO_NAMED(
      "rmw_zenoh_cpp",
      "[Publisher] Processing endpoint %zu/%zu with key='%s'",
      iteration, discovered_subscribers_.size(), sub.endpoint_key.c_str());

    auto endpoint_it = endpoints_.find(sub.endpoint_key);
    if (endpoint_it == endpoints_.end() || !endpoint_it->second) {
      continue;
    }
    auto endpoint = endpoint_it->second;

    size_t max_data_length = type_support_->get_estimated_serialized_size(
      ros_message, type_support_impl_);

    RMW_ZENOH_LOG_INFO_NAMED(
      "rmw_zenoh_cpp",
      "[Publisher] Estimated serialized size: %zu bytes", max_data_length);

    // Quadruple the buffer size for safety (endpoint-aware serialization needs more space)
    // TODO(native-buffer): Fix the size estimation in buffer_serialization.hpp to be more accurate
    max_data_length = max_data_length * 4 + 16384;

    RMW_ZENOH_LOG_INFO_NAMED(
      "rmw_zenoh_cpp",
      "[Publisher] Allocating buffer: %zu bytes (2x + 8KB safety margin)", max_data_length);

    rcutils_allocator_t * allocator = &rmw_node_->context->options.allocator;
    void * data = allocator->allocate(max_data_length, allocator->state);
    if (!data) {
      RMW_SET_ERROR_MSG("failed to allocate serialization buffer");
      return RMW_RET_BAD_ALLOC;
    }

    auto always_free_data = rcpputils::make_scope_exit(
      [data, allocator]() {
        allocator->deallocate(data, allocator->state);
      });

    uint8_t * msg_bytes = static_cast<uint8_t *>(data);
    eprosima::fastcdr::FastBuffer fastbuffer(reinterpret_cast<char *>(msg_bytes), max_data_length);
    rmw_zenoh_cpp::Cdr ser(fastbuffer);

    RMW_ZENOH_LOG_INFO_NAMED(
      "rmw_zenoh_cpp",
      "[Publisher] Starting endpoint-aware serialization...");

    rmw_zenoh_cpp::set_thread_local_backend_compatibility(&sub.backend_compat);
    bool ok = type_support_->serialize_ros_message_with_endpoint(
      ros_message, ser.get_cdr(), type_support_impl_, sub.endpoint_info.info);
    rmw_zenoh_cpp::set_thread_local_backend_compatibility(nullptr);
    if (!ok) {
      RMW_SET_ERROR_MSG("could not serialize ROS message with endpoint awareness");
      return RMW_RET_ERROR;
    }

    size_t data_length = ser.get_serialized_data_length();

    RMW_ZENOH_LOG_INFO_NAMED(
      "rmw_zenoh_cpp",
      "[Publisher] Serialization complete, actual size: %zu bytes (allocated: %zu, usage: %.1f%%)",
      data_length, max_data_length, (data_length * 100.0) / max_data_length);

    // Sanity check: ensure we didn't overflow
    if (data_length > max_data_length) {
      RMW_ZENOH_LOG_ERROR_NAMED(
        "rmw_zenoh_cpp",
        "[Publisher] CRITICAL: Serialized size %zu exceeds allocated buffer %zu!",
        data_length, max_data_length);
      return RMW_RET_ERROR;
    }

    // Publish to this endpoint
    // Use deleter to manage memory since Zenoh takes ownership
    auto deleter = [data, allocator](uint8_t *) {
        allocator->deallocate(data, allocator->state);
      };
    auto payload = zenoh::Bytes(msg_bytes, data_length, deleter);
    always_free_data.cancel();  // Zenoh now owns the memory

    // Create attachment AFTER serialization
    RMW_ZENOH_LOG_INFO_NAMED(
      "rmw_zenoh_cpp",
      "[Publisher] Creating attachment data...");

    int64_t source_timestamp = rmw_zenoh_cpp::get_system_time_in_ns();
    auto gid = entity_->copy_gid();

    RMW_ZENOH_LOG_INFO_NAMED(
      "rmw_zenoh_cpp",
      "[Publisher] Creating AttachmentData object...");

    auto attachment_data = rmw_zenoh_cpp::AttachmentData(
      sequence_number_++, source_timestamp, gid);

    RMW_ZENOH_LOG_INFO_NAMED(
      "rmw_zenoh_cpp",
      "[Publisher] Serializing attachment to zbytes...");

    auto attachment_bytes = attachment_data.serialize_to_zbytes();

    RMW_ZENOH_LOG_INFO_NAMED(
      "rmw_zenoh_cpp",
      "[Publisher] Attachment created successfully");

    zenoh::ext::AdvancedPublisher::PutOptions options =
      zenoh::ext::AdvancedPublisher::PutOptions::create_default();
    // Set attachment directly without std::make_optional (same as non-buffer-aware path)
    options.put_options.attachment = std::move(attachment_bytes);

    zenoh::ZResult result;
    if (endpoint->pub.has_value()) {
      RMW_ZENOH_LOG_INFO_NAMED(
        "rmw_zenoh_cpp",
        "[Publisher] Calling Zenoh put...");

      endpoint->pub.value().put(std::move(payload), std::move(options), &result);

      RMW_ZENOH_LOG_INFO_NAMED(
        "rmw_zenoh_cpp",
        "[Publisher] Zenoh put completed with result: %d", result);

      if (result != Z_OK) {
        RMW_ZENOH_LOG_ERROR_NAMED(
          "rmw_zenoh_cpp",
          "Failed to publish to endpoint '%s'", sub.endpoint_key.c_str());
        ret = RMW_RET_ERROR;
      }
    }
  }

  RMW_ZENOH_LOG_INFO_NAMED(
    "rmw_zenoh_cpp",
    "[Publisher] publish_buffer_aware() returning successfully");

  return ret;
}

///=============================================================================
rmw_ret_t PublisherData::publish(
  const void * ros_message,
  ShmContext * shm)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (is_shutdown_) {
    RMW_SET_ERROR_MSG("Unable to publish as the publisher has been shutdown.");
    return RMW_RET_ERROR;
  }

  // Buffer-aware publishers use different logic
  if (is_buffer_aware_) {
    return publish_buffer_aware(ros_message, shm);
  }

  // Serialize data.
  size_t max_data_length = type_support_->get_estimated_serialized_size(
    ros_message,
    type_support_impl_);

  // To store serialized message byte array.
  uint8_t * msg_bytes = nullptr;

  rmw_context_impl_s * context_impl =
    static_cast<rmw_context_impl_s *>(rmw_node_->data);
  if (context_impl == nullptr) {
    RMW_SET_ERROR_MSG("Unable to cast rmw_node->data into rmw_context_impl_s.");
    return RMW_RET_ERROR;
  }
  rcutils_allocator_t * allocator = &rmw_node_->context->options.allocator;

  // Optional shared memory buffer
  std::optional<zenoh::ZShmMut> shm_buf = std::nullopt;
  // Optional buffer reused for serialization from the buffer pool
  std::optional<BufferPool::Buffer> pool_buf = std::nullopt;

  auto always_free_msg_bytes = rcpputils::make_scope_exit(
    [&msg_bytes, allocator, &shm_buf, &pool_buf]() {
      if (!msg_bytes || shm_buf.has_value() || (pool_buf.has_value() && pool_buf.value().data)) {
        return;
      }
      allocator->deallocate(msg_bytes, allocator->state);
    });

  // Get memory from SHM buffer if available.
  if (shm && max_data_length >= shm->msgsize_threshold) {
    if (auto shm_provider = shm->get_shm_provider(*sess_)) {
      RMW_ZENOH_LOG_DEBUG_NAMED("rmw_zenoh_cpp", "SHM is enabled.");

      auto alloc_result = shm_provider.value().shm_provider().alloc_gc_defrag(max_data_length);

      if (std::holds_alternative<zenoh::ZShmMut>(alloc_result)) {
        auto && buf = std::get<zenoh::ZShmMut>(std::move(alloc_result));
        msg_bytes = reinterpret_cast<uint8_t *>(buf.data());
        shm_buf = std::make_optional(std::move(buf));
      } else {
        // Print a warning and revert to regular allocation
        RMW_ZENOH_LOG_DEBUG_NAMED(
          "rmw_zenoh_cpp", "Failed to allocate a SHM buffer, fallback to non-SHM");
      }
    } else {
      // Print a warning and revert to regular allocation
      RMW_ZENOH_LOG_DEBUG_NAMED(
        "rmw_zenoh_cpp", "SHM provider is not yet available, fallback to non-SHM");
    }
  }

  if (!shm_buf.has_value()) {
    // Try to get memory from the serialization buffer pool.
    pool_buf = context_impl->serialization_buffer_pool()->allocate(max_data_length);
    if (pool_buf.has_value() && pool_buf.value().data) {
      msg_bytes = pool_buf->data;
    } else {
      void * data = allocator->allocate(max_data_length, allocator->state);
      RMW_CHECK_FOR_NULL_WITH_MSG(
        data, "failed to allocate serialization buffer", return RMW_RET_BAD_ALLOC);
      msg_bytes = static_cast<uint8_t *>(data);
    }
  }

  RMW_CHECK_FOR_NULL_WITH_MSG(
    msg_bytes, "bytes for message is null", return RMW_RET_BAD_ALLOC);

  // Object that manages the raw buffer
  eprosima::fastcdr::FastBuffer fastbuffer(reinterpret_cast<char *>(msg_bytes), max_data_length);

  // Object that serializes the data
  rmw_zenoh_cpp::Cdr ser(fastbuffer);
  if (!type_support_->serialize_ros_message(
      ros_message,
      ser.get_cdr(),
      type_support_impl_))
  {
    RMW_SET_ERROR_MSG("could not serialize ROS message");
    return RMW_RET_ERROR;
  }

  const size_t data_length = ser.get_serialized_data_length();

  // The encoding is simply forwarded and is useful when key expressions in the
  // session use different encoding formats. In our case, all key expressions
  // will be encoded with CDR so it does not really matter.
  zenoh::ZResult result;
  int64_t source_timestamp = rmw_zenoh_cpp::get_system_time_in_ns();
  auto opts = zenoh::ext::AdvancedPublisher::PutOptions::create_default();
  opts.put_options.attachment = rmw_zenoh_cpp::AttachmentData(
    sequence_number_++, source_timestamp, entity_->copy_gid()).serialize_to_zbytes();

  zenoh::Bytes payload;
  if (shm_buf.has_value()) {
    payload = zenoh::Bytes(std::move(*shm_buf));
  } else if (pool_buf.has_value() && pool_buf.value().data) {
    auto deleter = [buffer_pool = context_impl->serialization_buffer_pool(),
        buffer = pool_buf](uint8_t *) {
        buffer_pool->deallocate(buffer.value());
      };
    payload = zenoh::Bytes(msg_bytes, data_length, deleter);
  } else {
    auto deleter = [msg_bytes, allocator](uint8_t *) {
        allocator->deallocate(msg_bytes, allocator->state);
      };
    payload = zenoh::Bytes(msg_bytes, data_length, deleter);
  }
  // The delete responsibility has been handed over to zenoh::Bytes now
  always_free_msg_bytes.cancel();

  TRACETOOLS_TRACEPOINT(
    rmw_publish, static_cast<const void *>(rmw_publisher_), ros_message, source_timestamp);
  pub_.put(std::move(payload), std::move(opts), &result);
  if (result != Z_OK) {
    if (result == Z_ESESSION_CLOSED) {
      RMW_ZENOH_LOG_WARN_NAMED(
        "rmw_zenoh_cpp",
        "unable to publish message since the zenoh session is closed");
    } else {
      RMW_SET_ERROR_MSG("unable to publish message");
      return RMW_RET_ERROR;
    }
  }

  return RMW_RET_OK;
}

///=============================================================================
rmw_ret_t PublisherData::publish_serialized_message(
  const rmw_serialized_message_t * serialized_message,
  ShmContext * shm)
{
  eprosima::fastcdr::FastBuffer buffer(
    reinterpret_cast<char *>(serialized_message->buffer), serialized_message->buffer_length);
  rmw_zenoh_cpp::Cdr ser(buffer);
  if (!ser.get_cdr().jump(serialized_message->buffer_length)) {
    RMW_SET_ERROR_MSG("cannot correctly set serialized buffer");
    return RMW_RET_ERROR;
  }

  // Optional shared memory buffer
  std::optional<zenoh::ZShmMut> shm_buf = std::nullopt;

  std::lock_guard<std::mutex> lock(mutex_);

  const size_t data_length = ser.get_serialized_data_length();
  // The encoding is simply forwarded and is useful when key expressions in the
  // session use different encoding formats. In our case, all key expressions
  // will be encoded with CDR so it does not really matter.
  zenoh::ZResult result;
  int64_t source_timestamp = rmw_zenoh_cpp::get_system_time_in_ns();
  auto opts = zenoh::ext::AdvancedPublisher::PutOptions::create_default();
  opts.put_options.attachment = rmw_zenoh_cpp::AttachmentData(
    sequence_number_++, source_timestamp, entity_->copy_gid()).serialize_to_zbytes();

  // Get memory from SHM buffer if available.
  if (shm && data_length >= shm->msgsize_threshold) {
    if (auto shm_provider = shm->get_shm_provider(*sess_)) {
      RMW_ZENOH_LOG_DEBUG_NAMED("rmw_zenoh_cpp", "SHM is enabled.");

      auto alloc_result = shm_provider.value().shm_provider().alloc_gc_defrag(data_length);

      if (std::holds_alternative<zenoh::ZShmMut>(alloc_result)) {
        auto && buf = std::get<zenoh::ZShmMut>(std::move(alloc_result));
        shm_buf = std::make_optional(std::move(buf));
      } else {
        // Print a warning and revert to regular allocation
        RMW_ZENOH_LOG_DEBUG_NAMED(
          "rmw_zenoh_cpp", "Failed to allocate a SHM buffer, fallback to non-SHM");
      }
    } else {
      // Print a warning and revert to regular allocation
      RMW_ZENOH_LOG_DEBUG_NAMED(
        "rmw_zenoh_cpp", "SHM provider is not yet available, fallback to non-SHM");
    }
  }

  if (shm_buf.has_value()) {
    auto msg_bytes = reinterpret_cast<char *>(shm_buf.value().data());
    memcpy(msg_bytes, serialized_message->buffer, data_length);
    zenoh::Bytes payload(std::move(*shm_buf));

    TRACETOOLS_TRACEPOINT(
      rmw_publish, static_cast<const void *>(rmw_publisher_), serialized_message,
      source_timestamp);

    pub_.put(std::move(payload), std::move(opts), &result);
  } else {
    std::vector<uint8_t> raw_image(
      serialized_message->buffer,
      serialized_message->buffer + data_length);
    zenoh::Bytes payload(std::move(raw_image));

    TRACETOOLS_TRACEPOINT(
      rmw_publish, static_cast<const void *>(rmw_publisher_), serialized_message,
        source_timestamp);

    pub_.put(std::move(payload), std::move(opts), &result);
  }

  if (result != Z_OK) {
    if (result == Z_ESESSION_CLOSED) {
      RMW_ZENOH_LOG_WARN_NAMED(
        "rmw_zenoh_cpp",
        "unable to publish message since the zenoh session is closed");
    } else {
      RMW_SET_ERROR_MSG("unable to publish message");
      return RMW_RET_ERROR;
    }
  }
  return RMW_RET_OK;
}

///=============================================================================
std::size_t PublisherData::gid_hash() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return entity_->gid_hash();
}

///=============================================================================
liveliness::TopicInfo PublisherData::topic_info() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return entity_->topic_info().value();
}

std::array<uint8_t, RMW_GID_STORAGE_SIZE> PublisherData::copy_gid() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return entity_->copy_gid();
}

///=============================================================================
bool PublisherData::liveliness_is_valid() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  // The z_check function is now internal in zenoh-1.0.0 so we assume
  // the liveliness token is still initialized as long as this entity has
  // not been shutdown.
  return !is_shutdown_;
}

///=============================================================================
std::shared_ptr<EventsManager> PublisherData::events_mgr() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return events_mgr_;
}

///=============================================================================
void PublisherData::on_subscriber_discovered(const liveliness::Entity & entity)
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (!is_buffer_aware_) {
    return;  // Simple publishers don't handle discovery
  }

  if (entity.type() != liveliness::EntityType::Subscription) {
    RMW_ZENOH_LOG_DEBUG_NAMED(
      "rmw_zenoh_cpp",
      "[Publisher] Ignoring discovered entity type=%s node='%s' ns='%s'",
      entity_type_to_string(entity.type()),
      entity.node_name().c_str(),
      entity.node_namespace().c_str());
    return;
  }

  // Get subscriber backend info (empty means CPU-only)
  auto topic_info_opt = entity.topic_info();
  if (!topic_info_opt.has_value()) {
    RMW_ZENOH_LOG_ERROR_NAMED(
      "rmw_zenoh_cpp",
      "Discovered subscriber without topic info on Buffer topic");
    return;
  }
  
  // Empty or missing backend_aux_info is treated as CPU-only
  std::unordered_map<std::string, std::string> sub_backend_aux_info;
  if (topic_info_opt->backend_aux_info_.has_value()) {
    sub_backend_aux_info = topic_info_opt->backend_aux_info_.value();
  }
  // If empty, it implicitly means CPU backend

  auto gid = entity_gid_to_rmw_gid(entity, rmw_zenoh_identifier);
  const auto entity_gid_array = entity.copy_gid();
  RMW_ZENOH_LOG_INFO_NAMED(
    "rmw_zenoh_cpp",
    "[Publisher] Discovered subscriber entity keyexpr='%s'",
    entity.liveliness_keyexpr().c_str());
  RMW_ZENOH_LOG_INFO_NAMED(
    "rmw_zenoh_cpp",
    "[Publisher] Discovered subscriber entity type=%s node='%s' ns='%s'",
    entity_type_to_string(entity.type()),
    entity.node_name().c_str(),
    entity.node_namespace().c_str());
  RMW_ZENOH_LOG_INFO_NAMED(
    "rmw_zenoh_cpp",
    "[Publisher] Discovered subscriber: zid='%s' gid='%s'",
    entity.zid().c_str(),
    gid_to_hex(gid).c_str());
  RMW_ZENOH_LOG_INFO_NAMED(
    "rmw_zenoh_cpp",
    "[Publisher] Discovered subscriber entity_gid='%s'",
    gid_array_to_hex(entity_gid_array).c_str());

  // Avoid duplicate registrations
  for (const auto & existing : discovered_subscribers_) {
    if (memcmp(existing.gid.data, gid.data, RMW_GID_STORAGE_SIZE) == 0) {
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

  auto sub_endpoint_info = build_endpoint_info_from_entity(entity, RMW_ENDPOINT_SUBSCRIPTION);

  std::vector<rmw_topic_endpoint_info_t> existing_endpoints;
  existing_endpoints.reserve(1 + discovered_subscribers_.size());
  existing_endpoints.push_back(local_endpoint_info_.info);
  for (const auto & existing : discovered_subscribers_) {
    existing_endpoints.push_back(existing.endpoint_info.info);
  }

  std::unordered_map<std::string, std::vector<std::set<uint32_t>>> backend_endpoint_groups;
  for (const auto & existing : discovered_subscribers_) {
    backend_endpoint_groups.insert(existing.backend_groups.begin(),
      existing.backend_groups.end());
  }

  std::unordered_map<std::string, std::vector<std::set<uint32_t>>> backend_groups;
  auto backend_compat = rmw_zenoh_cpp::inform_backends_on_discovering_endpoint(
    sub_endpoint_info.info, existing_endpoints, backend_endpoint_groups,
    sub_backend_aux_info);

  std::string full_key = entity_->topic_info()->topic_keyexpr_ + "/" +
    entity_->zid() + "/" + gid_to_hex(gid);

  if (endpoints_.find(full_key) == endpoints_.end()) {
    get_or_create_endpoint(full_key);
  }

  // Track subscriber
  SubscriberInfo sub_info;
  sub_info.gid = gid;
  sub_info.endpoint_key = full_key;
  sub_info.endpoint_info = std::move(sub_endpoint_info);
  sub_info.backend_aux_info = sub_backend_aux_info;
  sub_info.backend_compat = std::move(backend_compat);
  sub_info.backend_groups = std::move(backend_groups);
  discovered_subscribers_.push_back(std::move(sub_info));

  if (endpoints_.count(full_key)) {
    endpoints_[full_key]->target_subscribers.push_back(gid);
  }
}

///=============================================================================
std::shared_ptr<PublisherData::PublisherEndpoint> PublisherData::get_or_create_endpoint(
  const std::string & full_key)
{
  // Check if endpoint already exists
  auto it = endpoints_.find(full_key);
  if (it != endpoints_.end()) {
    return it->second;
  }

  // Create new endpoint
  auto endpoint = std::make_shared<PublisherEndpoint>();
  endpoint->key = full_key;

  // Create Zenoh publisher for this endpoint
  zenoh::KeyExpr pub_ke(full_key);

  // Copy QoS settings from the entity
  auto qos_profile = entity_->topic_info()->qos_;

  using AdvancedPublisherOptions = zenoh::ext::SessionExt::AdvancedPublisherOptions;
  auto adv_pub_opts = AdvancedPublisherOptions::create_default();

  if (qos_profile.durability == RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL) {
    adv_pub_opts.publisher_detection = true;
    adv_pub_opts.cache = AdvancedPublisherOptions::CacheOptions::create_default();
    adv_pub_opts.cache->max_samples = qos_profile.depth;
  }

  auto pub_opts = zenoh::Session::PublisherOptions::create_default();
  pub_opts.congestion_control = Z_CONGESTION_CONTROL_DROP;
  if (qos_profile.reliability == RMW_QOS_POLICY_RELIABILITY_RELIABLE) {
    pub_opts.reliability = Z_RELIABILITY_RELIABLE;
    if (qos_profile.history == RMW_QOS_POLICY_HISTORY_KEEP_ALL) {
      pub_opts.congestion_control = Z_CONGESTION_CONTROL_BLOCK;
    }
  } else {
    pub_opts.reliability = Z_RELIABILITY_BEST_EFFORT;
  }
  adv_pub_opts.publisher_options = pub_opts;

  zenoh::ZResult result;
  auto pub = sess_->ext().declare_advanced_publisher(
      pub_ke, std::move(adv_pub_opts), &result);

  if (result != Z_OK) {
    RMW_ZENOH_LOG_ERROR_NAMED(
      "rmw_zenoh_cpp",
      "Failed to create dynamic endpoint for key: %s", full_key.c_str());
    return nullptr;
  }

  endpoint->pub = std::optional<zenoh::ext::AdvancedPublisher>(std::move(pub));
  endpoints_[full_key] = endpoint;
  return endpoint;
}

///=============================================================================
PublisherData::~PublisherData()
{
  std::cerr << "[PublisherData] DESTRUCTOR ENTERED\n" << std::flush;
  const rmw_ret_t ret = this->shutdown();
  std::cerr << "[PublisherData] shutdown() returned with code: " << ret << "\n" << std::flush;
  if (ret != RMW_RET_OK) {
    RMW_ZENOH_LOG_ERROR_NAMED(
      "rmw_zenoh_cpp",
      "Error destructing publisher /%s.",
      entity_->topic_info().value().name_.c_str()
    );
  }
  std::cerr << "[PublisherData] DESTRUCTOR EXITING\n" << std::flush;
}

///=============================================================================
rmw_ret_t PublisherData::shutdown()
{
  std::cerr << "[PublisherData::shutdown] ENTERED\n" << std::flush;
  std::lock_guard<std::mutex> lock(mutex_);
  std::cerr << "[PublisherData::shutdown] Acquired mutex\n" << std::flush;
  if (is_shutdown_) {
    std::cerr << "[PublisherData::shutdown] Already shutdown\n" << std::flush;
    return RMW_RET_OK;
  }

  // Unregister discovery callbacks for buffer-aware publishers
  // NOTE: We skip this entirely to avoid deadlock with the graph subscriber callback thread
  // The callbacks will be cleared automatically when the RMW context is destroyed
  if (is_buffer_aware_ && graph_cache_ != nullptr) {
    std::cerr <<
      "[PublisherData::shutdown] Skipping discovery callback unregistration to avoid deadlock\n" <<
      std::flush;
    // graph_cache_->unregister_discovery_callbacks(gid_hash()); // DISABLED: causes deadlock
  }

  // Undeclare all dynamic endpoints for buffer-aware publishers
  if (is_buffer_aware_) {
    std::cerr << "[PublisherData::shutdown] Undeclaring " << endpoints_.size() <<
      " dynamic endpoints\n" << std::flush;
    zenoh::ZResult result;
    size_t endpoint_idx = 0;
    for (auto & [key, endpoint] : endpoints_) {
      std::cerr << "[PublisherData::shutdown] Processing endpoint " << (++endpoint_idx) << "/" <<
        endpoints_.size() << " key=" << key << "\n" << std::flush;
      if (endpoint->pub.has_value()) {
        std::cerr << "[PublisherData::shutdown] Undeclaring publisher for endpoint\n" << std::flush;
        std::move(endpoint->pub.value()).undeclare(&result);
        std::cerr << "[PublisherData::shutdown] Publisher undeclared with result: " << result <<
          "\n" << std::flush;
        if (result != Z_OK) {
          RMW_ZENOH_LOG_WARN_NAMED(
            "rmw_zenoh_cpp",
          "Failed to undeclare endpoint with key '%s'", key.c_str());
        }
      }
    }
    std::cerr << "[PublisherData::shutdown] Clearing endpoints map\n" << std::flush;
    endpoints_.clear();
    std::cerr << "[PublisherData::shutdown] Endpoints cleared\n" << std::flush;
  }

  // Unregister this publisher from the ROS graph.
  std::cerr << "[PublisherData::shutdown] Undeclaring liveliness token\n" << std::flush;
  zenoh::ZResult result;
  std::move(token_).value().undeclare(&result);
  std::cerr << "[PublisherData::shutdown] Liveliness token undeclared with result: " << result <<
    "\n" << std::flush;
  if (result != Z_OK) {
    RMW_ZENOH_LOG_ERROR_NAMED(
      "rmw_zenoh_cpp",
      "Unable to undeclare the liveliness token for topic '%s'",
      entity_->topic_info().value().name_.c_str());
    return RMW_RET_ERROR;
  }

  // For simple publishers, undeclare the base publisher
  if (!is_buffer_aware_) {
    std::cerr << "[PublisherData::shutdown] Undeclaring base publisher (simple)\n" << std::flush;
    std::move(pub_).undeclare(&result);
    std::cerr << "[PublisherData::shutdown] Base publisher undeclared\n" << std::flush;
  }
  if (result != Z_OK) {
    RMW_ZENOH_LOG_ERROR_NAMED(
      "rmw_zenoh_cpp",
      "Unable to undeclare the publisher for topic '%s'",
      entity_->topic_info().value().name_.c_str());
    return RMW_RET_ERROR;
  }

  std::cerr << "[PublisherData::shutdown] Resetting session\n" << std::flush;
  sess_.reset();
  std::cerr << "[PublisherData::shutdown] Session reset complete\n" << std::flush;
  is_shutdown_ = true;
  std::cerr << "[PublisherData::shutdown] EXITING successfully\n" << std::flush;
  return RMW_RET_OK;
}

///=============================================================================
bool PublisherData::is_shutdown() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return is_shutdown_;
}
}  // namespace rmw_zenoh_cpp
