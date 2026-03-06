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

namespace rmw_zenoh_cpp
{

/// Initialize buffer backend system.
/// Loads buffer backend plugins and registers them with FastCDR serialization.
void initialize_buffer_backends();

/// Cleanup buffer backend system.
/// Clears global serialization maps to release plugin references before unloading.
void shutdown_buffer_backends();

}  // namespace rmw_zenoh_cpp

#endif  // RMW_ZENOH_CPP__BUFFER_BACKEND_LOADER_HPP_
