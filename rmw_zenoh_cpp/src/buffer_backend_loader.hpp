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

#ifndef RMW_ZENOH_CPP__BUFFER_BACKEND_LOADER_HPP_
#define RMW_ZENOH_CPP__BUFFER_BACKEND_LOADER_HPP_

#include <string>
#include <vector>

#include "rmw/types.h"

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

/// Compute the Zenoh key suffix based on locality and backend compatibility
/// @param locality The locality of the remote endpoint
/// @param pub_backends Publisher's supported backends
/// @param sub_backends Subscriber's supported backends
/// @return Key suffix string (e.g., "cpu", "ipc_cuda", "inter_process_cuda")
std::string compute_endpoint_key_suffix(
  rmw_endpoint_locality_t locality,
  const std::vector<std::string> & pub_backends,
  const std::vector<std::string> & sub_backends);

}  // namespace rmw_zenoh_cpp

#endif  // RMW_ZENOH_CPP__BUFFER_BACKEND_LOADER_HPP_
