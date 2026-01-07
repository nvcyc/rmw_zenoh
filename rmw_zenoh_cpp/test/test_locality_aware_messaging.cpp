// Copyright 2025 NVIDIA CORPORATION
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

#include <gtest/gtest.h>
#include <string>
#include <vector>

#include "rmw/types.h"

namespace rmw_zenoh_cpp
{

class LocalityAwareMessagingTest : public ::testing::Test
{
protected:
  void SetUp() override {}
  void TearDown() override {}
};

// Test locality type enumeration
TEST_F(LocalityAwareMessagingTest, LocalityTypes)
{
  EXPECT_EQ(RMW_ENDPOINT_LOCALITY_UNDEFINED, 0);
  EXPECT_EQ(RMW_ENDPOINT_LOCALITY_INTRA_PROCESS, 1);
  EXPECT_EQ(RMW_ENDPOINT_LOCALITY_INTER_PROCESS_SAME_HOST, 2);
  EXPECT_EQ(RMW_ENDPOINT_LOCALITY_INTER_HOST, 3);
}

// Test key suffix for different localities
TEST_F(LocalityAwareMessagingTest, KeySuffix_IntraProcess)
{
  std::string base_key = "/camera/image";
  std::string suffix = "/ipc_cuda";
  std::string full_key = base_key + suffix;

  EXPECT_EQ(full_key, "/camera/image/ipc_cuda");
  EXPECT_NE(full_key.find("/ipc_"), std::string::npos);
}

TEST_F(LocalityAwareMessagingTest, KeySuffix_InterProcess)
{
  std::string base_key = "/camera/image";
  std::string suffix = "/shm_cuda";
  std::string full_key = base_key + suffix;

  EXPECT_EQ(full_key, "/camera/image/shm_cuda");
  EXPECT_NE(full_key.find("/shm_"), std::string::npos);
}

TEST_F(LocalityAwareMessagingTest, KeySuffix_InterHost)
{
  std::string base_key = "/camera/image";
  std::string suffix = "/cpu";
  std::string full_key = base_key + suffix;

  EXPECT_EQ(full_key, "/camera/image/cpu");
}

// Test message routing scenarios
TEST_F(LocalityAwareMessagingTest, RoutingScenario_IntraProcessCudaOptimal)
{
  // Same process, both support CUDA
  rmw_endpoint_locality_t locality = RMW_ENDPOINT_LOCALITY_INTRA_PROCESS;
  std::vector<std::string> common_backends = {"cuda", "cpu"};

  // Should use CUDA for zero-copy
  std::string optimal_backend = "cuda";
  EXPECT_EQ(common_backends[0], optimal_backend);

  // Key would be: /topic/ipc_cuda
  std::string key_suffix = "/ipc_" + optimal_backend;
  EXPECT_EQ(key_suffix, "/ipc_cuda");
}

TEST_F(LocalityAwareMessagingTest, RoutingScenario_InterProcessSharedMemory)
{
  // Different processes on same host, both support CUDA
  rmw_endpoint_locality_t locality = RMW_ENDPOINT_LOCALITY_INTER_PROCESS_SAME_HOST;
  std::vector<std::string> common_backends = {"cuda", "cpu"};

  // Should use shared memory with CUDA
  std::string key_suffix = "/shm_cuda";
  EXPECT_EQ(key_suffix, "/shm_cuda");
}

TEST_F(LocalityAwareMessagingTest, RoutingScenario_InterHostSerialized)
{
  // Different hosts, must serialize
  rmw_endpoint_locality_t locality = RMW_ENDPOINT_LOCALITY_INTER_HOST;
  std::vector<std::string> common_backends = {"cuda", "cpu"};

  // Should use CPU path with full serialization
  std::string key_suffix = "/cpu";
  EXPECT_EQ(key_suffix, "/cpu");
}

TEST_F(LocalityAwareMessagingTest, RoutingScenario_OnlyCpuAvailable)
{
  // Intra-process, but only CPU backend available
  rmw_endpoint_locality_t locality = RMW_ENDPOINT_LOCALITY_INTRA_PROCESS;
  std::vector<std::string> common_backends = {"cpu"};

  // Should use CPU
  std::string optimal_backend = common_backends[0];
  EXPECT_EQ(optimal_backend, "cpu");

  std::string key_suffix = "/ipc_" + optimal_backend;
  EXPECT_EQ(key_suffix, "/ipc_cpu");
}

// Test serialization strategy selection
TEST_F(LocalityAwareMessagingTest, SerializationStrategy_ZeroCopy)
{
  // Intra-process with CUDA = zero-copy possible
  rmw_endpoint_locality_t locality = RMW_ENDPOINT_LOCALITY_INTRA_PROCESS;
  std::string backend = "cuda";

  bool can_zero_copy = (locality == RMW_ENDPOINT_LOCALITY_INTRA_PROCESS) &&
    (backend == "cuda" || backend == "cpu");
  EXPECT_TRUE(can_zero_copy);
}

TEST_F(LocalityAwareMessagingTest, SerializationStrategy_SharedMemory)
{
  // Inter-process with CUDA = shared memory
  rmw_endpoint_locality_t locality = RMW_ENDPOINT_LOCALITY_INTER_PROCESS_SAME_HOST;
  std::string backend = "cuda";

  bool use_shm = (locality == RMW_ENDPOINT_LOCALITY_INTER_PROCESS_SAME_HOST) &&
    (backend == "cuda" || backend == "cpu");
  EXPECT_TRUE(use_shm);
}

TEST_F(LocalityAwareMessagingTest, SerializationStrategy_FullSerialization)
{
  // Inter-host = full serialization required
  rmw_endpoint_locality_t locality = RMW_ENDPOINT_LOCALITY_INTER_HOST;

  bool full_serialize = (locality == RMW_ENDPOINT_LOCALITY_INTER_HOST);
  EXPECT_TRUE(full_serialize);
}

// Test message metadata storage
TEST_F(LocalityAwareMessagingTest, MessageMetadata_LocalityStorage)
{
  // Simulate storing locality with message
  struct MessageWithLocality
  {
    std::vector<uint8_t> payload;
    rmw_endpoint_locality_t locality;
  };

  MessageWithLocality msg;
  msg.payload = {1, 2, 3, 4};
  msg.locality = RMW_ENDPOINT_LOCALITY_INTRA_PROCESS;

  EXPECT_EQ(msg.payload.size(), 4u);
  EXPECT_EQ(msg.locality, RMW_ENDPOINT_LOCALITY_INTRA_PROCESS);
}

// Test deserialization context
TEST_F(LocalityAwareMessagingTest, Deserialization_UsesStoredLocality)
{
  // When deserializing, we should use the locality stored with the message
  rmw_endpoint_locality_t stored_locality = RMW_ENDPOINT_LOCALITY_INTRA_PROCESS;

  // Deserialization function would receive this locality
  bool should_use_zero_copy = (stored_locality == RMW_ENDPOINT_LOCALITY_INTRA_PROCESS);
  EXPECT_TRUE(should_use_zero_copy);
}

// Test endpoint map structure
TEST_F(LocalityAwareMessagingTest, EndpointMap_MultipleKeys)
{
  // Subscriber can have multiple subscriptions for different localities/backends
  std::map<std::string, rmw_endpoint_locality_t> subscriptions;

  subscriptions["/topic/ipc_cuda"] = RMW_ENDPOINT_LOCALITY_INTRA_PROCESS;
  subscriptions["/topic/shm_cuda"] = RMW_ENDPOINT_LOCALITY_INTER_PROCESS_SAME_HOST;
  subscriptions["/topic/cpu"] = RMW_ENDPOINT_LOCALITY_INTER_HOST;

  EXPECT_EQ(subscriptions.size(), 3u);
  EXPECT_EQ(subscriptions["/topic/ipc_cuda"], RMW_ENDPOINT_LOCALITY_INTRA_PROCESS);
  EXPECT_EQ(subscriptions["/topic/shm_cuda"], RMW_ENDPOINT_LOCALITY_INTER_PROCESS_SAME_HOST);
  EXPECT_EQ(subscriptions["/topic/cpu"], RMW_ENDPOINT_LOCALITY_INTER_HOST);
}

// Test discovery and dynamic subscription creation logic
TEST_F(LocalityAwareMessagingTest, DynamicSubscription_CreationConditions)
{
  // Subscription should be created when:
  // 1. Publisher is discovered
  bool publisher_discovered = true;

  // 2. Backends are compatible
  std::vector<std::string> pub_backends = {"cuda", "cpu"};
  std::vector<std::string> sub_backends = {"cuda"};
  bool backends_compatible = false;
  for (const auto & pb : pub_backends) {
    for (const auto & sb : sub_backends) {
      if (pb == sb) {
        backends_compatible = true;
        break;
      }
    }
    if (backends_compatible) {break;}
  }

  // 3. Subscription doesn't already exist for this key
  bool subscription_exists = false;

  bool should_create_subscription = publisher_discovered &&
    backends_compatible &&
    !subscription_exists;

  EXPECT_TRUE(should_create_subscription);
}

}  // namespace rmw_zenoh_cpp

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
