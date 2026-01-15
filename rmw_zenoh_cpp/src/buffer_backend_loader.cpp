// Copyright 2024 NVIDIA Corporation
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

#include "rosidl_buffer_registry/buffer_backend_registry.hpp"
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

  // Load all available buffer backends via pluginlib into the generic registry
  // Each backend is completely serialization-independent
  try {
    std::cerr << "[RMW Zenoh] Loading buffer backend plugins via pluginlib...\n";
    rosidl_buffer_registry::BufferBackendRegistry::get_instance().load_plugins();
  } catch (const std::exception & e) {
    std::cerr << "[RMW Zenoh] Plugin loading exception: " << e.what() << "\n";
    // Non-fatal: No buffer backend plugins found
    // This is expected on systems without vendor buffer support (CPU-only systems)
  }

  // Populate global maps in rosidl_typesupport_fastrtps_cpp
  // Map 1: Backend descriptor operations (technology-independent)
  // Map 2: FastCDR descriptor serializers (technology-specific)
  auto & generic_registry = rosidl_buffer_registry::BufferBackendRegistry::get_instance();
  std::cerr << "[RMW Zenoh] Generic registry instance at: " << &generic_registry << "\n";

  auto & backend_ops = rosidl_typesupport_fastrtps_cpp::get_backend_descriptor_ops();
  std::cerr << "[RMW Zenoh] Backend ops map at: " << &backend_ops << "\n";

  auto backend_names = generic_registry.get_backend_names();
  std::cerr << "[RMW Zenoh] Found " << backend_names.size() << " backend(s)\n";

  for (const auto & backend_name : backend_names) {
    std::cerr << "[RMW Zenoh] Processing backend: " << backend_name << "\n";

    auto backend = generic_registry.get_backend(backend_name);
    if (!backend) {
      std::cerr << "[RMW Zenoh]   ERROR: Backend pointer is null!\n";
      continue;
    }

    std::string backend_type = backend->get_backend_type();
    std::cerr << "[RMW Zenoh]   Backend type: " << backend_type << "\n";
    std::cerr << "[RMW Zenoh]   Descriptor type: " << backend->get_descriptor_type_name() << "\n";

    // Populate backend descriptor operations map
    rosidl_typesupport_fastrtps_cpp::BackendDescriptorOps ops;
    ops.descriptor_type_name = backend->get_descriptor_type_name();

    auto backend_ptr = backend;  // Capture for lambdas
    ops.create_descriptor = [backend_ptr](
      const std::shared_ptr<void> & impl) -> std::shared_ptr<void> {
        return backend_ptr->create_descriptor(impl);
      };
    ops.from_descriptor = [backend_ptr](
      const std::shared_ptr<void> & descriptor) -> std::shared_ptr<void> {
        return backend_ptr->from_descriptor(descriptor);
      };
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
    std::cerr << "[RMW Zenoh]   ✓ Registered backend ops for: " << backend_type << "\n";

    // Call FastCDR registration function to populate serializers map
    std::cerr << "[RMW Zenoh]   Getting FastCDR registration function...\n";
    void * reg_func_ptr = backend->get_descriptor_registration_function();

    if (reg_func_ptr) {
      std::cerr << "[RMW Zenoh]   Found registration function at " << reg_func_ptr <<
        ", calling it...\n";
      auto register_func = reinterpret_cast<RegisterDescriptorFunc>(reg_func_ptr);
      register_func();
      std::cerr << "[RMW Zenoh]   ✓ Successfully called FastCDR registration\n";
    } else {
      std::cerr << "[RMW Zenoh]   ✗ Backend does not provide FastCDR registration function\n";
    }
  }

  std::cerr << "[RMW Zenoh] Buffer backend initialization complete\n";
  std::cerr << "[RMW Zenoh] Total backends registered: " << backend_ops.size() << "\n";

  // Register endpoint compatibility resolver for endpoint-aware serialization.
  rosidl_typesupport_fastrtps_cpp::get_endpoint_compatibility_resolver() =
    [](const rmw_topic_endpoint_info_t &, const std::string & backend_type) {
      return rmw_zenoh_cpp::get_thread_local_backend_compatibility(backend_type);
    };
}

void shutdown_buffer_backends()
{
  std::cerr << "[RMW Zenoh] Shutting down buffer backends...\n";

  // Clear global serialization maps that hold lambdas capturing backend shared_ptrs
  // This MUST be done before BufferBackendRegistry singleton is destroyed
  // to prevent ClassLoader from trying to unload while objects exist
  try {
    auto & backend_ops = rosidl_typesupport_fastrtps_cpp::get_backend_descriptor_ops();
    backend_ops.clear();
    std::cerr << "[RMW Zenoh] Cleared backend descriptor ops\n";

    auto & serializers = rosidl_typesupport_fastrtps_cpp::get_descriptor_serializers();
    serializers.clear();
    std::cerr << "[RMW Zenoh] Cleared descriptor serializers\n";
  } catch (const std::exception & e) {
    std::cerr << "[RMW Zenoh] Warning during buffer backend shutdown: " << e.what() << "\n";
  }

  // Clear the backend registry to release shared_ptr to plugin instances
  try {
    rosidl_buffer_registry::BufferBackendRegistry::get_instance().clear_global_state();
    std::cerr << "[RMW Zenoh] Cleared buffer backend registry\n";
  } catch (const std::exception & e) {
    std::cerr << "[RMW Zenoh] Warning clearing backend registry: " << e.what() << "\n";
  }

  std::cerr << "[RMW Zenoh] Buffer backend shutdown complete\n";
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
    auto & registry = rosidl_buffer_registry::BufferBackendRegistry::get_instance();
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
    std::cerr << "[RMW Zenoh] Warning getting backend types: " << e.what() << "\n";
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
std::unordered_map<std::string, std::string> collect_backend_aux_info(
  const rmw_topic_endpoint_info_t & endpoint_info,
  const std::vector<std::string> & backend_types)
{
  std::unordered_map<std::string, std::string> aux_info;
  aux_info.reserve(backend_types.size());

  auto & registry = rosidl_buffer_registry::BufferBackendRegistry::get_instance();
  for (const auto & backend_type : backend_types) {
    if (backend_type == "cpu") {
      aux_info[backend_type] = "";
      continue;
    }
    auto backend = registry.get_backend(backend_type);
    if (!backend) {
      aux_info[backend_type] = "";
      continue;
    }
    aux_info[backend_type] = backend->on_creating_endpoint(endpoint_info);
  }

  return aux_info;
}

///=============================================================================
std::unordered_map<std::string, bool> evaluate_backend_compatibility(
  const rmw_topic_endpoint_info_t & endpoint_info,
  const std::vector<rmw_topic_endpoint_info_t> & existing_endpoints,
  std::unordered_map<std::string, std::vector<std::set<uint32_t>>> & backend_groups)
{
  std::unordered_map<std::string, bool> compat;
  auto backend_types = get_installed_backend_types();
  compat.reserve(backend_types.size());

  auto & registry = rosidl_buffer_registry::BufferBackendRegistry::get_instance();
  for (const auto & backend_type : backend_types) {
    if (backend_type == "cpu") {
      compat[backend_type] = true;
      backend_groups[backend_type] = {};
      continue;
    }
    auto backend = registry.get_backend(backend_type);
    if (!backend) {
      compat[backend_type] = false;
      backend_groups[backend_type] = {};
      continue;
    }

    auto result = backend->on_discovering_endpoint(endpoint_info, existing_endpoints);
    compat[backend_type] = result.first;
    backend_groups[backend_type] = std::move(result.second);
  }

  return compat;
}

}  // namespace rmw_zenoh_cpp
