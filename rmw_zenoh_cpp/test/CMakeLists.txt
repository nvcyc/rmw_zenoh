# Copyright 2025 NVIDIA CORPORATION
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

cmake_minimum_required(VERSION 3.5)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  
  # Test for buffer backend helper functions
  ament_add_gtest(test_buffer_backend_helpers
    test_buffer_backend_helpers.cpp
  )
  if(TARGET test_buffer_backend_helpers)
    target_link_libraries(test_buffer_backend_helpers
      ${PROJECT_NAME}
    )
    ament_target_dependencies(test_buffer_backend_helpers
      rmw
      rosidl_buffer_registry
    )
    target_include_directories(test_buffer_backend_helpers PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/../src
    )
  endif()
  
  # Test for subscription creation and decision logic
  ament_add_gtest(test_subscription_creation
    test_subscription_creation.cpp
  )
  if(TARGET test_subscription_creation)
    target_link_libraries(test_subscription_creation
      ${PROJECT_NAME}
    )
    ament_target_dependencies(test_subscription_creation
      rmw
      rosidl_buffer_registry
    )
    target_include_directories(test_subscription_creation PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/../src
    )
  endif()
  
  # Test for liveliness key with backend information
  ament_add_gtest(test_liveliness_backends
    test_liveliness_backends.cpp
  )
  if(TARGET test_liveliness_backends)
    target_link_libraries(test_liveliness_backends
      ${PROJECT_NAME}
    )
    ament_target_dependencies(test_liveliness_backends
      rmw
      rosidl_buffer_registry
    )
    target_include_directories(test_liveliness_backends PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/../src
    )
  endif()
  
  # Test for locality-aware message handling
  ament_add_gtest(test_locality_aware_messaging
    test_locality_aware_messaging.cpp
  )
  if(TARGET test_locality_aware_messaging)
    target_link_libraries(test_locality_aware_messaging
      ${PROJECT_NAME}
    )
    ament_target_dependencies(test_locality_aware_messaging
      rmw
      rosidl_buffer_registry
    )
    target_include_directories(test_locality_aware_messaging PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/../src
    )
  endif()

endif()
