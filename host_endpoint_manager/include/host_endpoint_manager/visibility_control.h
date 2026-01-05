// Copyright 2025 Open Source Robotics Foundation, Inc.
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

#ifndef HOST_ENDPOINT_MANAGER__VISIBILITY_CONTROL_H_
#define HOST_ENDPOINT_MANAGER__VISIBILITY_CONTROL_H_

#ifdef __cplusplus
extern "C"
{
#endif

// This logic was borrowed from the file rosidl_generator_c/resource/msg__type_support.h.em
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define HOST_ENDPOINT_MANAGER_EXPORT __attribute__ ((dllexport))
    #define HOST_ENDPOINT_MANAGER_IMPORT __attribute__ ((dllimport))
  #else
    #define HOST_ENDPOINT_MANAGER_EXPORT __declspec(dllexport)
    #define HOST_ENDPOINT_MANAGER_IMPORT __declspec(dllimport)
  #endif
  #ifdef HOST_ENDPOINT_MANAGER_BUILDING_DLL
    #define HOST_ENDPOINT_MANAGER_PUBLIC HOST_ENDPOINT_MANAGER_EXPORT
  #else
    #define HOST_ENDPOINT_MANAGER_PUBLIC HOST_ENDPOINT_MANAGER_IMPORT
  #endif
  #define HOST_ENDPOINT_MANAGER_PUBLIC_TYPE HOST_ENDPOINT_MANAGER_PUBLIC
  #define HOST_ENDPOINT_MANAGER_LOCAL
#else
  #define HOST_ENDPOINT_MANAGER_EXPORT __attribute__ ((visibility("default")))
  #define HOST_ENDPOINT_MANAGER_IMPORT
  #if __GNUC__ >= 4
    #define HOST_ENDPOINT_MANAGER_PUBLIC __attribute__ ((visibility("default")))
    #define HOST_ENDPOINT_MANAGER_PUBLIC_TYPE __attribute__ ((visibility("default")))
    #define HOST_ENDPOINT_MANAGER_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define HOST_ENDPOINT_MANAGER_PUBLIC
    #define HOST_ENDPOINT_MANAGER_PUBLIC_TYPE
    #define HOST_ENDPOINT_MANAGER_LOCAL
  #endif
#endif

#ifdef __cplusplus
}
#endif

#endif  // HOST_ENDPOINT_MANAGER__VISIBILITY_CONTROL_H_
