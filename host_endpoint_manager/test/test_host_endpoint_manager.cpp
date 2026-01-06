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

#include <gtest/gtest.h>

#include <cstring>
#include <memory>
#include <thread>
#include <vector>

#include "host_endpoint_manager/host_endpoint_manager.hpp"
#include "rmw/types.h"

using host_endpoint_manager::HostEndpointManager;
using host_endpoint_manager::EntityType;
using host_endpoint_manager::LocalityInfo;

class TestHostEndpointManager : public ::testing::Test
{
protected:
  void SetUp() override
  {
    // Use a unique domain for each test to avoid interference
    static size_t test_domain = 100;
    domain_id_ = test_domain++;
  }

  void TearDown() override
  {
    // Release all references to allow cleanup
    managers_.clear();
  }

  rmw_gid_t create_test_gid(uint64_t value)
  {
    rmw_gid_t gid;
    gid.implementation_identifier = "test_rmw";
    std::memset(gid.data, 0, RMW_GID_STORAGE_SIZE);
    std::memcpy(gid.data, &value, sizeof(value));
    return gid;
  }

  size_t domain_id_;
  std::vector<std::shared_ptr<HostEndpointManager>> managers_;
};

//==============================================================================
TEST_F(TestHostEndpointManager, SingletonSameDomain)
{
  // Multiple get_instance calls with same domain should return same instance
  auto mgr1 = HostEndpointManager::get_instance(domain_id_);
  auto mgr2 = HostEndpointManager::get_instance(domain_id_);

  ASSERT_NE(mgr1, nullptr);
  ASSERT_NE(mgr2, nullptr);
  EXPECT_EQ(mgr1, mgr2);  // Same pointer
  EXPECT_EQ(mgr1->get_instance_id(), mgr2->get_instance_id());
  EXPECT_EQ(mgr1->get_domain_id(), domain_id_);

  managers_.push_back(mgr1);
}

//==============================================================================
TEST_F(TestHostEndpointManager, SingletonDifferentDomains)
{
  // Different domains should get different instances
  auto mgr1 = HostEndpointManager::get_instance(domain_id_);
  auto mgr2 = HostEndpointManager::get_instance(domain_id_ + 1);

  ASSERT_NE(mgr1, nullptr);
  ASSERT_NE(mgr2, nullptr);
  EXPECT_NE(mgr1, mgr2);  // Different pointers
  EXPECT_NE(mgr1->get_instance_id(), mgr2->get_instance_id());

  managers_.push_back(mgr1);
  managers_.push_back(mgr2);
}

//==============================================================================
TEST_F(TestHostEndpointManager, RegisterPublisher)
{
  auto mgr = HostEndpointManager::get_instance(domain_id_);
  managers_.push_back(mgr);

  rmw_gid_t gid = create_test_gid(1);
  EXPECT_TRUE(mgr->register_publisher(gid, "test_topic"));

  // Query should find it as intra-process
  auto info = mgr->query_endpoint_locality(gid);
  EXPECT_TRUE(info.found);
  EXPECT_EQ(info.locality, RMW_ENDPOINT_LOCALITY_INTRA_PROCESS);
  EXPECT_EQ(info.entity_type, EntityType::PUBLISHER);
  EXPECT_EQ(info.remote_instance_id, mgr->get_instance_id());
}

//==============================================================================
TEST_F(TestHostEndpointManager, RegisterSubscription)
{
  auto mgr = HostEndpointManager::get_instance(domain_id_);
  managers_.push_back(mgr);

  rmw_gid_t gid = create_test_gid(2);
  EXPECT_TRUE(mgr->register_subscription(gid, "test_topic"));

  auto info = mgr->query_endpoint_locality(gid);
  EXPECT_TRUE(info.found);
  EXPECT_EQ(info.locality, RMW_ENDPOINT_LOCALITY_INTRA_PROCESS);
  EXPECT_EQ(info.entity_type, EntityType::SUBSCRIPTION);
}

//==============================================================================
TEST_F(TestHostEndpointManager, RegisterServiceClient)
{
  auto mgr = HostEndpointManager::get_instance(domain_id_);
  managers_.push_back(mgr);

  rmw_gid_t gid = create_test_gid(3);
  EXPECT_TRUE(mgr->register_service_client(gid, "test_service"));

  auto info = mgr->query_endpoint_locality(gid);
  EXPECT_TRUE(info.found);
  EXPECT_EQ(info.locality, RMW_ENDPOINT_LOCALITY_INTRA_PROCESS);
  EXPECT_EQ(info.entity_type, EntityType::SERVICE_CLIENT);
}

//==============================================================================
TEST_F(TestHostEndpointManager, RegisterServiceServer)
{
  auto mgr = HostEndpointManager::get_instance(domain_id_);
  managers_.push_back(mgr);

  rmw_gid_t gid = create_test_gid(4);
  EXPECT_TRUE(mgr->register_service_server(gid, "test_service"));

  auto info = mgr->query_endpoint_locality(gid);
  EXPECT_TRUE(info.found);
  EXPECT_EQ(info.locality, RMW_ENDPOINT_LOCALITY_INTRA_PROCESS);
  EXPECT_EQ(info.entity_type, EntityType::SERVICE_SERVER);
}

//==============================================================================
TEST_F(TestHostEndpointManager, UnregisterEndpoint)
{
  auto mgr = HostEndpointManager::get_instance(domain_id_);
  managers_.push_back(mgr);

  rmw_gid_t gid = create_test_gid(5);
  EXPECT_TRUE(mgr->register_publisher(gid, "test_topic"));

  // Should find it
  auto info = mgr->query_endpoint_locality(gid);
  EXPECT_TRUE(info.found);

  // Unregister
  EXPECT_TRUE(mgr->unregister_endpoint(gid));

  // Should not find it anymore
  info = mgr->query_endpoint_locality(gid);
  EXPECT_FALSE(info.found);
}

//==============================================================================
TEST_F(TestHostEndpointManager, QueryNonExistentEndpoint)
{
  auto mgr = HostEndpointManager::get_instance(domain_id_);
  managers_.push_back(mgr);

  rmw_gid_t gid = create_test_gid(999);

  auto info = mgr->query_endpoint_locality(gid);
  EXPECT_FALSE(info.found);
  EXPECT_EQ(info.locality, RMW_ENDPOINT_LOCALITY_UNDEFINED);
}

//==============================================================================
TEST_F(TestHostEndpointManager, MultipleEndpoints)
{
  auto mgr = HostEndpointManager::get_instance(domain_id_);
  managers_.push_back(mgr);

  // Register multiple endpoints
  rmw_gid_t gid1 = create_test_gid(10);
  rmw_gid_t gid2 = create_test_gid(11);
  rmw_gid_t gid3 = create_test_gid(12);

  EXPECT_TRUE(mgr->register_publisher(gid1, "topic1"));
  EXPECT_TRUE(mgr->register_subscription(gid2, "topic2"));
  EXPECT_TRUE(mgr->register_service_client(gid3, "service1"));

  // All should be found
  EXPECT_TRUE(mgr->query_endpoint_locality(gid1).found);
  EXPECT_TRUE(mgr->query_endpoint_locality(gid2).found);
  EXPECT_TRUE(mgr->query_endpoint_locality(gid3).found);

  // Check stats
  auto stats = mgr->get_stats();
  EXPECT_EQ(stats.local_cache_size, 3u);
  EXPECT_GE(stats.shm_entries, 3u);
  EXPECT_EQ(stats.shm_capacity, 512u);
}

//==============================================================================
TEST_F(TestHostEndpointManager, RefreshFromRemote)
{
  auto mgr = HostEndpointManager::get_instance(domain_id_);
  managers_.push_back(mgr);

  rmw_gid_t gid = create_test_gid(20);
  EXPECT_TRUE(mgr->register_publisher(gid, "test_topic"));

  // Refresh should not break anything
  mgr->refresh_from_remote();

  // Should still be found
  auto info = mgr->query_endpoint_locality(gid);
  EXPECT_TRUE(info.found);
  EXPECT_EQ(info.locality, RMW_ENDPOINT_LOCALITY_INTRA_PROCESS);
}

//==============================================================================
TEST_F(TestHostEndpointManager, ThreadSafeRegistration)
{
  auto mgr = HostEndpointManager::get_instance(domain_id_);
  managers_.push_back(mgr);

  const size_t num_threads = 10;
  const size_t endpoints_per_thread = 10;
  std::vector<std::thread> threads;

  // Register endpoints from multiple threads
  for (size_t t = 0; t < num_threads; ++t) {
    threads.emplace_back(
      [&mgr, t, endpoints_per_thread]() {
        for (size_t i = 0; i < endpoints_per_thread; ++i) {
          uint64_t gid_value = t * 1000 + i;
          rmw_gid_t gid;
          gid.implementation_identifier = "test";
          std::memset(gid.data, 0, RMW_GID_STORAGE_SIZE);
          std::memcpy(gid.data, &gid_value, sizeof(gid_value));

          mgr->register_publisher(gid, "thread_topic");
        }
      });
  }

  for (auto & thread : threads) {
    thread.join();
  }

  // All endpoints should be registered
  auto stats = mgr->get_stats();
  EXPECT_EQ(stats.local_cache_size, num_threads * endpoints_per_thread);
}

//==============================================================================
TEST_F(TestHostEndpointManager, ThreadSafeQuery)
{
  auto mgr = HostEndpointManager::get_instance(domain_id_);
  managers_.push_back(mgr);

  // Register some endpoints
  std::vector<rmw_gid_t> gids;
  for (size_t i = 0; i < 100; ++i) {
    rmw_gid_t gid = create_test_gid(i);
    mgr->register_publisher(gid, "test_topic");
    gids.push_back(gid);
  }

  // Query from multiple threads simultaneously (lock-free)
  const size_t num_threads = 20;
  std::vector<std::thread> threads;
  std::atomic<size_t> success_count{0};

  for (size_t t = 0; t < num_threads; ++t) {
    threads.emplace_back(
      [&mgr, &gids, &success_count]() {
        for (size_t i = 0; i < 1000; ++i) {
          size_t idx = i % gids.size();
          auto info = mgr->query_endpoint_locality(gids[idx]);
          if (info.found) {
            success_count++;
          }
        }
      });
  }

  for (auto & thread : threads) {
    thread.join();
  }

  // All queries should have succeeded
  EXPECT_EQ(success_count.load(), num_threads * 1000);
}

//==============================================================================
TEST_F(TestHostEndpointManager, HostnameRetrieval)
{
  auto mgr = HostEndpointManager::get_instance(domain_id_);
  managers_.push_back(mgr);

  std::string hostname = mgr->get_hostname();
  EXPECT_FALSE(hostname.empty());
  EXPECT_GT(hostname.length(), 0u);
}

//==============================================================================
TEST_F(TestHostEndpointManager, UpdateExistingEndpoint)
{
  auto mgr = HostEndpointManager::get_instance(domain_id_);
  managers_.push_back(mgr);

  rmw_gid_t gid = create_test_gid(30);

  // Register as publisher
  EXPECT_TRUE(mgr->register_publisher(gid, "topic1"));
  auto info = mgr->query_endpoint_locality(gid);
  EXPECT_EQ(info.entity_type, EntityType::PUBLISHER);

  // Re-register as subscription (update)
  EXPECT_TRUE(mgr->register_subscription(gid, "topic2"));
  info = mgr->query_endpoint_locality(gid);
  EXPECT_EQ(info.entity_type, EntityType::SUBSCRIPTION);
}

//==============================================================================
TEST_F(TestHostEndpointManager, MultiContextIntraProcess)
{
  // Simulate multiple RMW contexts in same process
  auto mgr1 = HostEndpointManager::get_instance(domain_id_);
  auto mgr2 = HostEndpointManager::get_instance(domain_id_);

  EXPECT_EQ(mgr1, mgr2);  // Same singleton
  EXPECT_EQ(mgr1->get_instance_id(), mgr2->get_instance_id());

  // Register from "context 1"
  rmw_gid_t pub_gid = create_test_gid(40);
  mgr1->register_publisher(pub_gid, "topic");

  // Register from "context 2"
  rmw_gid_t sub_gid = create_test_gid(41);
  mgr2->register_subscription(sub_gid, "topic");

  // Both should detect each other as intra-process
  auto pub_info = mgr1->query_endpoint_locality(sub_gid);
  EXPECT_TRUE(pub_info.found);
  EXPECT_EQ(pub_info.locality, RMW_ENDPOINT_LOCALITY_INTRA_PROCESS);

  auto sub_info = mgr2->query_endpoint_locality(pub_gid);
  EXPECT_TRUE(sub_info.found);
  EXPECT_EQ(sub_info.locality, RMW_ENDPOINT_LOCALITY_INTRA_PROCESS);

  managers_.push_back(mgr1);
}

//==============================================================================
int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

