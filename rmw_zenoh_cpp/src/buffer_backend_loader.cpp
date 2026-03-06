// Copyright 2026 Open Source Robotics Foundation, Inc.
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

#include "buffer_backend_loader.hpp"

#include <iostream>
#include <memory>
#include <string>

#include "detail/logging_macros.hpp"
#include "rcl_buffer_backend_registry/buffer_backend_registry.hpp"
#include "rosidl_typesupport_fastrtps_cpp/buffer_serialization.hpp"

namespace rmw_zenoh_cpp
{

void initialize_buffer_backends()
{
  std::cerr << "[RMW Zenoh] Initializing buffer backends...\n";

  // Note: CPU backend doesn't need registration - it's handled directly
  // in buffer_serialization.hpp by serializing as std::vector<T>

  // Load all available buffer backends via pluginlib into the buffer backend registry
  // Each backend is completely serialization-independent
  auto & registry = rcl_buffer_backend_registry::BufferBackendRegistry::get_instance();
  registry.load_plugins();

  // Populate global maps in rosidl_typesupport_fastrtps_cpp
  // Map 1: Backend descriptor operations (technology-independent)
  // Map 2: FastCDR descriptor serializers (technology-specific)

  auto & backend_ops = rosidl_typesupport_fastrtps_cpp::get_backend_descriptor_ops();
  auto backend_names = registry.get_backend_names();
  RMW_ZENOH_LOG_INFO_NAMED("rmw_zenoh_cpp", "Found %d backend(s)", backend_names.size());

  for (const auto & backend_name : backend_names) {
    RMW_ZENOH_LOG_INFO_NAMED("rmw_zenoh_cpp", "Processing backend: %s", backend_name.c_str());

    auto backend = registry.get_backend(backend_name);
    if (!backend) {
      RMW_ZENOH_LOG_ERROR_NAMED("rmw_zenoh_cpp", "Backend pointer is null!");
      continue;
    }

    std::string backend_type = backend->get_backend_type();
    RMW_ZENOH_LOG_INFO_NAMED("rmw_zenoh_cpp", "  Descriptor type: %s", backend->get_descriptor_type_name().c_str());

    // Populate backend descriptor operations map
    rosidl_typesupport_fastrtps_cpp::BackendDescriptorOps ops;
    ops.descriptor_type_name = backend->get_descriptor_type_name();

    auto backend_ptr = backend;  // Capture for lambdas
    ops.create_descriptor_with_endpoint = [backend_ptr](
      const std::shared_ptr<void> & impl,
      const rmw_topic_endpoint_info_t & endpoint_info) -> std::shared_ptr<void> {
        return backend_ptr->create_descriptor_with_endpoint(impl, endpoint_info);
      };
    ops.from_descriptor_with_endpoint = [backend_ptr](
      const std::shared_ptr<void> & descriptor,
      const rmw_topic_endpoint_info_t & endpoint_info) -> std::shared_ptr<void> {
        return backend_ptr->from_descriptor_with_endpoint(descriptor, endpoint_info);
      };

    backend_ops[backend_type] = ops;

    // Verify that the backend registered its FastCDR descriptor serializers
    // (backends auto-register in their constructor via register_buffer_descriptor<T>())
    auto & serializers = rosidl_typesupport_fastrtps_cpp::get_descriptor_serializers();
    if (serializers.find(backend_type) != serializers.end()) {
      RMW_ZENOH_LOG_INFO_NAMED(
        "rmw_zenoh_cpp", "  FastCDR descriptor serializers registered for '%s'",
        backend_type.c_str());
    } else {
      RMW_ZENOH_LOG_ERROR_NAMED(
        "rmw_zenoh_cpp",
        "  Backend '%s' did not register FastCDR descriptor serializers. "
        "Ensure the backend constructor calls "
        "rcl_buffer::register_buffer_descriptor<DescriptorMsgT>()",
        backend_type.c_str());
    }
  }

}

void shutdown_buffer_backends()
{
  // Clear global serialization maps that hold lambdas capturing backend shared_ptrs
  // This MUST be done before BufferBackendRegistry singleton is destroyed
  // to prevent ClassLoader from trying to unload while objects exist
  try {
    auto & backend_ops = rosidl_typesupport_fastrtps_cpp::get_backend_descriptor_ops();
    backend_ops.clear();

    auto & serializers = rosidl_typesupport_fastrtps_cpp::get_descriptor_serializers();
    serializers.clear();
  } catch (const std::exception & e) {
    RMW_ZENOH_LOG_ERROR_NAMED("rmw_zenoh_cpp", "Warning during buffer backend shutdown: %s", e.what());
  }

  // Clear the backend registry to release shared_ptr to plugin instances
  try {
    rcl_buffer_backend_registry::BufferBackendRegistry::get_instance().clear_global_state();
  } catch (const std::exception & e) {
    RMW_ZENOH_LOG_ERROR_NAMED("rmw_zenoh_cpp", "Warning clearing backend registry: %s", e.what());
  }

}

}  // namespace rmw_zenoh_cpp
