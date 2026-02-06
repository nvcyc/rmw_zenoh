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

#ifndef RMW_ZENOH_CPP__BUFFER_BACKEND_LOADER_HPP_
#define RMW_ZENOH_CPP__BUFFER_BACKEND_LOADER_HPP_

#include <set>
#include <string>
#include <unordered_map>
#include <vector>

#include "rmw/topic_endpoint_info.h"

namespace rmw_zenoh_cpp
{

/// Initialize buffer backend system
/// Loads buffer backend plugins and registers them with FastCDR serialization
void initialize_buffer_backends();

/// Cleanup buffer backend system
/// Clears global serialization maps to release plugin references before unloading
void shutdown_buffer_backends();

/// Get list of installed backend type strings
/// @return Vector of backend type names (e.g., ["cuda", "cpu"])
std::vector<std::string> get_installed_backend_types();

/// Check if two backend lists have at least one common backend
/// @param a First backend list
/// @param b Second backend list
/// @return true if at least one common backend exists
bool backends_compatible(
  const std::vector<std::string> & a,
  const std::vector<std::string> & b);

/// Get the intersection of two backend lists
/// @param a First backend list
/// @param b Second backend list
/// @return Vector of common backends
std::vector<std::string> get_common_backends(
  const std::vector<std::string> & a,
  const std::vector<std::string> & b);

/// Collect backend aux info from loaded backends.
std::unordered_map<std::string, std::string> collect_backend_aux_info();

/// Inform all backends on creating an endpoint.
void inform_backends_on_creating_endpoint(
  const rmw_topic_endpoint_info_t & endpoint_info);

/// Inform all backends on discovering an endpoint.
/// @param endpoint_info Information about the discovered endpoint
/// @param existing_endpoints List of existing endpoints for grouping decisions
/// @param backend_groups Output parameter for backend-specific grouping information
/// @param endpoint_supported_backends Backend aux info from the discovered endpoint
std::unordered_map<std::string, bool> inform_backends_on_discovering_endpoint(
  const rmw_topic_endpoint_info_t & endpoint_info,
  const std::vector<rmw_topic_endpoint_info_t> & existing_endpoints,
  std::unordered_map<std::string, std::vector<std::set<uint32_t>>> & backend_groups,
  const std::unordered_map<std::string, std::string> & endpoint_supported_backends);

/// Set thread-local backend compatibility map for serialization.
void set_thread_local_backend_compatibility(
  const std::unordered_map<std::string, bool> * compat_map);

/// Query thread-local backend compatibility for a backend type.
bool get_thread_local_backend_compatibility(const std::string & backend_type);

}  // namespace rmw_zenoh_cpp

#endif  // RMW_ZENOH_CPP__BUFFER_BACKEND_LOADER_HPP_
