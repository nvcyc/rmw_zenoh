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
#include <memory>
#include <mutex>
#include <optional>
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

#include "host_endpoint_manager/host_endpoint_manager.hpp"

#include "rcpputils/scope_exit.hpp"

#include "rmw/error_handling.h"
#include "rmw/get_topic_endpoint_info.h"
#include "rmw/impl/cpp/macros.hpp"

#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"

#include "tracetools/tracetools.h"

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
  std::optional<std::vector<std::string>> backend_types = std::nullopt;
  std::vector<std::string> my_backend_types;
  if (is_buffer_aware) {
    my_backend_types = rmw_zenoh_cpp::get_installed_backend_types();
    backend_types = my_backend_types;
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
      backend_types}  // Include backends only if Buffer message type
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
      my_backend_types
    });

  // Register with Host Endpoint Manager
  auto context_impl = static_cast<rmw_context_impl_t *>(node->context->impl);
  auto endpoint_manager = context_impl->endpoint_manager();
  if (endpoint_manager != nullptr) {
    rmw_gid_t gid = rmw_zenoh_cpp::entity_gid_to_rmw_gid(
      *pub_data->entity_, rmw_zenoh_cpp::rmw_zenoh_identifier);

    if (!endpoint_manager->register_publisher(gid, topic_name.c_str())) {
      RMW_ZENOH_LOG_ERROR_NAMED(
        "rmw_zenoh_cpp",
        "Failed to register publisher with Host Endpoint Manager");
      return nullptr;
    }
  }

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
  std::vector<std::string> my_backend_types)
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
  my_backend_types_(std::move(my_backend_types))
{
  events_mgr_ = std::make_shared<EventsManager>();

  // For simple publishers, create a single base endpoint
  if (!is_buffer_aware_) {
    auto base_endpoint = std::make_shared<PublisherEndpoint>();
    base_endpoint->key_suffix = "";
    base_endpoint->full_key = entity_->topic_info()->topic_keyexpr_;
    base_endpoint->pub = std::optional<zenoh::ext::AdvancedPublisher>(std::move(pub_));
    endpoints_[""] = base_endpoint;
  }
  // For buffer-aware publishers, endpoints are created dynamically on subscriber discovery
}

///=============================================================================
// Helper function for buffer-aware publishing
rmw_ret_t PublisherData::publish_buffer_aware(
  const void * ros_message,
  ShmContext * shm)
{
  // For buffer-aware publishers, route to different endpoints based on discovered subscribers
  if (discovered_subscribers_.empty()) {
    // No subscribers yet, skip publish
    return RMW_RET_OK;
  }

  // Group subscribers by endpoint suffix
  std::unordered_map<std::string, std::vector<SubscriberInfo *>> groups;
  for (auto & sub : discovered_subscribers_) {
    groups[sub.assigned_endpoint_key].push_back(&sub);
  }

  // Clear message caches
  for (auto & [key, ep] : endpoints_) {
    ep->cached_message.reset();
  }

  // Serialize and publish to each endpoint
  rmw_ret_t ret = RMW_RET_OK;
  size_t iteration = 0;
  for (auto & [key_suffix, subs] : groups) {
    iteration++;
    RMW_ZENOH_LOG_INFO_NAMED(
      "rmw_zenoh_cpp",
      "[Publisher] Processing endpoint group %zu/%zu with key_suffix='%s'",
      iteration, groups.size(), key_suffix.c_str());

    auto endpoint = endpoints_[key_suffix];
    if (!endpoint) {
      continue;
    }

    // Determine locality for this endpoint group
    // All subscribers in this group have the same locality by construction
    rmw_endpoint_locality_t locality = RMW_ENDPOINT_LOCALITY_UNDEFINED;
    if (!subs.empty() && subs[0] != nullptr) {
      locality = subs[0]->locality;
    }

    // Serialize data using locality-aware serialization
    size_t max_data_length = type_support_->get_estimated_serialized_size(
      ros_message, type_support_impl_);

    RMW_ZENOH_LOG_INFO_NAMED(
      "rmw_zenoh_cpp",
      "[Publisher] Estimated serialized size: %zu bytes", max_data_length);

    // Quadruple the buffer size for safety (locality-aware serialization needs more space)
    // TODO: Fix the size estimation in buffer_serialization.hpp to be more accurate
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
      "[Publisher] Starting locality-aware serialization...");

    // Use locality-aware serialization for Buffer-aware messages
    if (!type_support_->serialize_ros_message_with_locality(
      ros_message, ser.get_cdr(), type_support_impl_, locality))
    {
      RMW_SET_ERROR_MSG("could not serialize ROS message with locality awareness");
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
          "Failed to publish to endpoint with suffix '%s'", key_suffix.c_str());
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

  // Check if subscriber has backend types (should always be true for Buffer topics)
  auto topic_info_opt = entity.topic_info();
  if (!topic_info_opt.has_value() || !topic_info_opt->backend_types_.has_value()) {
    RMW_ZENOH_LOG_WARN_NAMED(
      "rmw_zenoh_cpp",
      "Discovered subscriber without backend types on Buffer topic");
    return;
  }

  auto sub_backends = topic_info_opt->backend_types_.value();
  auto gid = entity_gid_to_rmw_gid(entity, rmw_zenoh_identifier);

  // Check backend compatibility
  if (!rmw_zenoh_cpp::backends_compatible(my_backend_types_, sub_backends)) {
    RMW_ZENOH_LOG_WARN_NAMED(
      "rmw_zenoh_cpp",
      "Incompatible backends between publisher and subscriber");
    return;
  }

  // Query locality from Host Endpoint Manager
  auto context_impl = static_cast<rmw_context_impl_t *>(rmw_node_->context->impl);
  auto endpoint_manager = context_impl->endpoint_manager();

  rmw_endpoint_locality_t locality = RMW_ENDPOINT_LOCALITY_UNDEFINED;
  if (endpoint_manager != nullptr) {
    auto locality_info = endpoint_manager->query_endpoint_locality(gid);
    locality = locality_info.locality;
  }

  // Compute key suffix based on locality and backends
  std::string key_suffix = rmw_zenoh_cpp::compute_endpoint_key_suffix(
    locality, my_backend_types_, sub_backends);

  std::string full_key = entity_->topic_info()->topic_keyexpr_ + "/" + key_suffix;

  // Create endpoint if not exists
  if (endpoints_.find(key_suffix) == endpoints_.end()) {
    get_or_create_endpoint(key_suffix, full_key);
  }

  // Track subscriber
  SubscriberInfo sub_info;
  sub_info.gid = gid;
  sub_info.locality = locality;
  sub_info.backend_types = sub_backends;
  sub_info.assigned_endpoint_key = key_suffix;
  discovered_subscribers_.push_back(sub_info);

  if (endpoints_.count(key_suffix)) {
    endpoints_[key_suffix]->target_subscribers.push_back(gid);
  }
}

///=============================================================================
std::shared_ptr<PublisherData::PublisherEndpoint> PublisherData::get_or_create_endpoint(
  const std::string & key_suffix,
  const std::string & full_key)
{
  // Check if endpoint already exists
  auto it = endpoints_.find(key_suffix);
  if (it != endpoints_.end()) {
    return it->second;
  }

  // Create new endpoint
  auto endpoint = std::make_shared<PublisherEndpoint>();
  endpoint->key_suffix = key_suffix;
  endpoint->full_key = full_key;

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
      "Failed to create dynamic endpoint for key suffix: %s", key_suffix.c_str());
    return nullptr;
  }

  endpoint->pub = std::optional<zenoh::ext::AdvancedPublisher>(std::move(pub));
  endpoints_[key_suffix] = endpoint;
  return endpoint;
}

///=============================================================================
PublisherData::~PublisherData()
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
rmw_ret_t PublisherData::shutdown()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (is_shutdown_) {
    return RMW_RET_OK;
  }

  // Unregister discovery callbacks for buffer-aware publishers
  if (is_buffer_aware_ && graph_cache_ != nullptr) {
    graph_cache_->unregister_discovery_callbacks(gid_hash());
  }

  // Undeclare all dynamic endpoints for buffer-aware publishers
  if (is_buffer_aware_) {
    zenoh::ZResult result;
    for (auto & [key_suffix, endpoint] : endpoints_) {
      if (endpoint->pub.has_value()) {
        std::move(endpoint->pub.value()).undeclare(&result);
        if (result != Z_OK) {
          RMW_ZENOH_LOG_WARN_NAMED(
            "rmw_zenoh_cpp",
            "Failed to undeclare endpoint with suffix '%s'", key_suffix.c_str());
        }
      }
    }
    endpoints_.clear();
  }

  // Unregister this publisher from the ROS graph.
  zenoh::ZResult result;
  std::move(token_).value().undeclare(&result);
  if (result != Z_OK) {
    RMW_ZENOH_LOG_ERROR_NAMED(
      "rmw_zenoh_cpp",
      "Unable to undeclare the liveliness token for topic '%s'",
      entity_->topic_info().value().name_.c_str());
    return RMW_RET_ERROR;
  }

  // For simple publishers, undeclare the base publisher
  if (!is_buffer_aware_) {
    std::move(pub_).undeclare(&result);
  }
  if (result != Z_OK) {
    RMW_ZENOH_LOG_ERROR_NAMED(
      "rmw_zenoh_cpp",
      "Unable to undeclare the publisher for topic '%s'",
      entity_->topic_info().value().name_.c_str());
    return RMW_RET_ERROR;
  }

  // Unregister from Host Endpoint Manager
  auto context_impl = static_cast<rmw_context_impl_t *>(rmw_node_->context->impl);
  auto endpoint_manager = context_impl->endpoint_manager();
  if (endpoint_manager != nullptr) {
    rmw_gid_t gid = rmw_zenoh_cpp::entity_gid_to_rmw_gid(
      *entity_, rmw_zenoh_cpp::rmw_zenoh_identifier);

    if (!endpoint_manager->unregister_endpoint(gid)) {
      RMW_ZENOH_LOG_ERROR_NAMED(
        "rmw_zenoh_cpp",
        "Failed to unregister publisher from Host Endpoint Manager");
      return RMW_RET_ERROR;
    }
  }

  sess_.reset();
  is_shutdown_ = true;
  return RMW_RET_OK;
}

///=============================================================================
bool PublisherData::is_shutdown() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return is_shutdown_;
}
}  // namespace rmw_zenoh_cpp
