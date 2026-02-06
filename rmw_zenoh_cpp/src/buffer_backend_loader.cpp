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

namespace
{
thread_local const std::unordered_map<std::string, bool> * g_tls_backend_compat = nullptr;
}  // namespace

// Function pointer type for descriptor registration functions
using RegisterDescriptorFunc = void (*)();

void initialize_buffer_backends()
{
  std::cerr << "[RMW Zenoh] Initializing buffer backends...\n";

  // Note: CPU backend doesn't need registration - it's handled directly
  // in buffer_serialization.hpp by serializing as std::vector<T>

  // Load all available buffer backends via pluginlib into the buffer backend registry
  // Each backend is completely serialization-independent
  auto & buffer_backend_registry = rcl_buffer_backend_registry::BufferBackendRegistry::get_instance();
  buffer_backend_registry.load_plugins();

  // Populate global maps in rosidl_typesupport_fastrtps_cpp
  // Map 1: Backend descriptor operations (technology-independent)
  // Map 2: FastCDR descriptor serializers (technology-specific)

  auto & backend_ops = rosidl_typesupport_fastrtps_cpp::get_backend_descriptor_ops();
  auto backend_names = buffer_backend_registry.get_backend_names();
  RMW_ZENOH_LOG_INFO_NAMED("rmw_zenoh_cpp", "Found %d backend(s)", backend_names.size());

  for (const auto & backend_name : backend_names) {
    RMW_ZENOH_LOG_INFO_NAMED("rmw_zenoh_cpp", "Processing backend: %s", backend_name.c_str());

    auto backend = buffer_backend_registry.get_backend(backend_name);
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

    // Call FastCDR registration function to populate serializers map
    void * reg_func_ptr = backend->get_descriptor_registration_function();

    if (reg_func_ptr) {
      auto register_func = reinterpret_cast<RegisterDescriptorFunc>(reg_func_ptr);
      register_func();
      RMW_ZENOH_LOG_INFO_NAMED("rmw_zenoh_cpp", "  Successfully called FastCDR registration");
    } else {
      RMW_ZENOH_LOG_ERROR_NAMED("rmw_zenoh_cpp", "  Backend does not provide FastCDR registration function");
    }
  }

  // Register endpoint compatibility resolver for endpoint-aware serialization.
  rosidl_typesupport_fastrtps_cpp::get_endpoint_compatibility_resolver() =
    [](const rmw_topic_endpoint_info_t &, const std::string & backend_type) {
      return rmw_zenoh_cpp::get_thread_local_backend_compatibility(backend_type);
    };
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

  rosidl_typesupport_fastrtps_cpp::get_endpoint_compatibility_resolver() = nullptr;
}

///=============================================================================
void set_thread_local_backend_compatibility(
  const std::unordered_map<std::string, bool> * compat_map)
{
  g_tls_backend_compat = compat_map;
}

///=============================================================================
bool get_thread_local_backend_compatibility(const std::string & backend_type)
{
  if (!g_tls_backend_compat) {
    return true;
  }
  auto it = g_tls_backend_compat->find(backend_type);
  if (it == g_tls_backend_compat->end()) {
    return true;
  }
  return it->second;
}

///=============================================================================
std::vector<std::string> get_installed_backend_types()
{
  std::vector<std::string> backend_types;

  // Always include CPU backend
  backend_types.push_back("cpu");

  // Get additional backends from the registry
  try {
    auto & registry = rcl_buffer_backend_registry::BufferBackendRegistry::get_instance();
    auto backend_names = registry.get_backend_names();

    for (const auto & backend_name : backend_names) {
      auto backend = registry.get_backend(backend_name);
      if (backend) {
        std::string backend_type = backend->get_backend_type();
        // Avoid duplicating CPU if it's in the registry
        if (backend_type != "cpu") {
          backend_types.push_back(backend_type);
        }
      }
    }
  } catch (const std::exception & e) {
    RMW_ZENOH_LOG_ERROR_NAMED("rmw_zenoh_cpp", "Warning getting backend types: %s", e.what());
  }

  return backend_types;
}

///=============================================================================
bool backends_compatible(
  const std::vector<std::string> & a,
  const std::vector<std::string> & b)
{
  // Check if there's at least one common backend
  for (const auto & backend_a : a) {
    for (const auto & backend_b : b) {
      if (backend_a == backend_b) {
        return true;
      }
    }
  }
  return false;
}

///=============================================================================
std::vector<std::string> get_common_backends(
  const std::vector<std::string> & a,
  const std::vector<std::string> & b)
{
  std::vector<std::string> common;

  for (const auto & backend_a : a) {
    for (const auto & backend_b : b) {
      if (backend_a == backend_b) {
        // Check if not already in common
        bool already_added = false;
        for (const auto & c : common) {
          if (c == backend_a) {
            already_added = true;
            break;
          }
        }
        if (!already_added) {
          common.push_back(backend_a);
        }
      }
    }
  }

  return common;
}

///=============================================================================
std::unordered_map<std::string, std::string> collect_backend_aux_info()
{
  std::unordered_map<std::string, std::string> aux_info;

  auto & registry = rcl_buffer_backend_registry::BufferBackendRegistry::get_instance();
  for (const auto & backend_name : registry.get_backend_names()) {
    auto backend = registry.get_backend(backend_name);
    if (!backend) {
      continue;
    }
    aux_info[backend_name] = backend->get_backend_aux_info();
  }

  return aux_info;
}

///=============================================================================
void inform_backends_on_creating_endpoint(
  const rmw_topic_endpoint_info_t & endpoint_info)
{
  auto & registry = rcl_buffer_backend_registry::BufferBackendRegistry::get_instance();
  for (const auto & backend_name : registry.get_backend_names()) {
    auto backend = registry.get_backend(backend_name);
    if (!backend) {
      continue;
    }
    backend->on_creating_endpoint(endpoint_info);
  }
}

///=============================================================================
std::unordered_map<std::string, bool> inform_backends_on_discovering_endpoint(
  const rmw_topic_endpoint_info_t & endpoint_info,
  const std::vector<rmw_topic_endpoint_info_t> & existing_endpoints,
  std::unordered_map<std::string, std::vector<std::set<uint32_t>>> & backend_endpoint_groups,
  const std::unordered_map<std::string, std::string> & endpoint_supported_backends)
{
  std::unordered_map<std::string, bool> backend_compatibility;
  auto & registry = rcl_buffer_backend_registry::BufferBackendRegistry::get_instance();
  for (const auto & backend_name : registry.get_backend_names()) {
    auto backend = registry.get_backend(backend_name);
    if (!backend) {
      backend_compatibility[backend_name] = false;
      backend_endpoint_groups[backend_name] = {};
      continue;
    }
    auto result = backend->on_discovering_endpoint(
      endpoint_info, existing_endpoints, endpoint_supported_backends);
    backend_compatibility[backend_name] = result.first;
    backend_endpoint_groups[backend_name] = std::move(result.second);
  }
  return backend_compatibility;
}

}  // namespace rmw_zenoh_cpp
