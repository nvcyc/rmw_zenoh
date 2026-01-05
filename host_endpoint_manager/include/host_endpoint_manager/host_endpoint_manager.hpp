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

#ifndef HOST_ENDPOINT_MANAGER__HOST_ENDPOINT_MANAGER_HPP_
#define HOST_ENDPOINT_MANAGER__HOST_ENDPOINT_MANAGER_HPP_

#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include "host_endpoint_manager/visibility_control.h"
#include "rmw/types.h"

namespace host_endpoint_manager
{

/// Entity type enumeration - RMW-agnostic classification
enum class EntityType : uint8_t
{
  PUBLISHER = 0,        ///< Message Publisher
  SUBSCRIPTION = 1,     ///< Message Subscription
  SERVICE_CLIENT = 2,   ///< Service Client
  SERVICE_SERVER = 3    ///< Service Server
};

/// Locality information for an endpoint
struct LocalityInfo
{
  rmw_endpoint_locality_t locality;  ///< Locality classification
  EntityType entity_type;             ///< Type of the endpoint
  uint64_t remote_instance_id;        ///< Instance ID (for debugging)
  bool found;                         ///< false = not on this host
};

/// Statistics about the manager state
struct Stats
{
  size_t local_cache_size;  ///< Number of entries in local cache
  size_t shm_entries;       ///< Active entries in shared memory
  size_t shm_capacity;      ///< Maximum SHM capacity
};

/// Host Endpoint Manager - Singleton per (process, domain) pair
/**
 * Tracks endpoint locality for transport optimization in RMW implementations.
 *
 * Key features:
 * - Singleton per domain: Multiple RMW contexts share the same instance
 * - Lock-free queries: O(1) lookup with no IPC overhead
 * - Same-host only: Tracks intra-process and inter-process same-host endpoints
 * - RMW-agnostic: Works with any RMW implementation
 *
 * Thread safety:
 * - Singleton creation: Thread-safe
 * - Registration: Thread-safe (uses SHM mutex)
 * - Queries: Lock-free (read-only cache access)
 * - Refresh: Thread-safe (uses SHM + cache mutexes)
 */
class HOST_ENDPOINT_MANAGER_PUBLIC HostEndpointManager
{
public:
  /// Get or create singleton instance for the given domain
  /**
   * Multiple calls with the same domain_id from the same process return
   * the same instance, ensuring correct intra-process detection.
   *
   * \param domain_id ROS domain ID
   * \return Shared pointer to the singleton instance
   * \throws std::runtime_error if initialization fails
   */
  static std::shared_ptr<HostEndpointManager> get_instance(size_t domain_id);

  /// Destructor - cleans up shared memory resources
  ~HostEndpointManager();

  // Deleted copy/move operations (singleton pattern)
  HostEndpointManager(const HostEndpointManager &) = delete;
  HostEndpointManager & operator=(const HostEndpointManager &) = delete;
  HostEndpointManager(HostEndpointManager &&) = delete;
  HostEndpointManager & operator=(HostEndpointManager &&) = delete;

  /// Register a publisher endpoint
  /**
   * \param gid Publisher GID
   * \param topic_name Topic name (for debugging)
   * \return true if registered successfully, false if registry full
   */
  bool register_publisher(const rmw_gid_t & gid, const char * topic_name);

  /// Register a subscription endpoint
  /**
   * \param gid Subscription GID
   * \param topic_name Topic name (for debugging)
   * \return true if registered successfully, false if registry full
   */
  bool register_subscription(const rmw_gid_t & gid, const char * topic_name);

  /// Register a service client endpoint
  /**
   * \param gid Service client GID
   * \param service_name Service name (for debugging)
   * \return true if registered successfully, false if registry full
   */
  bool register_service_client(const rmw_gid_t & gid, const char * service_name);

  /// Register a service server endpoint
  /**
   * \param gid Service server GID
   * \param service_name Service name (for debugging)
   * \return true if registered successfully, false if registry full
   */
  bool register_service_server(const rmw_gid_t & gid, const char * service_name);

  /// Unregister an endpoint of any type
  /**
   * \param gid Endpoint GID to unregister
   * \return true if unregistered successfully, false if not found
   */
  bool unregister_endpoint(const rmw_gid_t & gid);

  /// Query endpoint locality (lock-free)
  /**
   * Looks up the endpoint in the local cache without any locking.
   *
   * \param gid Endpoint GID to query
   * \return Locality information
   *         - found=false: Endpoint not on this host (or not yet discovered)
   *         - locality=INTRA_PROCESS: Same instance_id (same process/domain)
   *         - locality=INTER_PROCESS_SAME_HOST: Different instance_id but same host
   */
  LocalityInfo query_endpoint_locality(const rmw_gid_t & gid) const;

  /// Refresh local cache from shared memory
  /**
   * Should be called when RMW detects new remote endpoints via graph events.
   * Scans shared memory and updates the local cache.
   */
  void refresh_from_remote();

  /// Refresh specific endpoints (optimization)
  /**
   * \param gids Vector of GIDs to refresh
   */
  void refresh_endpoints(const std::vector<rmw_gid_t> & gids);

  /// Get this manager's unique instance ID
  /**
   * \return Instance ID (unique per process-domain pair)
   */
  uint64_t get_instance_id() const;

  /// Get the domain ID this manager tracks
  /**
   * \return Domain ID
   */
  size_t get_domain_id() const;

  /// Get the hostname this manager uses
  /**
   * \return Hostname string
   */
  std::string get_hostname() const;

  /// Get statistics about manager state
  /**
   * \return Statistics structure
   */
  Stats get_stats() const;

private:
  /// Private constructor (singleton pattern)
  explicit HostEndpointManager(size_t domain_id);

  /// Helper: Register an endpoint with specified type
  bool register_endpoint(
    const rmw_gid_t & gid,
    const char * name,
    EntityType type);

  /// Helper: Hash a GID to size_t for map lookup
  static size_t hash_gid(const rmw_gid_t & gid);

  /// Helper: Hash a raw GID byte array
  static size_t hash_gid_raw(const uint8_t * gid);

  /// Helper: Hash a string to uint32_t
  static uint32_t hash_string(const std::string & str);

  // Singleton instance storage
  static std::mutex instances_mutex_;
  static std::unordered_map<size_t, std::weak_ptr<HostEndpointManager>> instances_;

  // Instance members
  const size_t domain_id_;
  const uint64_t instance_id_;
  std::string hostname_;

  // Shared memory resources
  int shm_fd_;
  void * shm_ptr_;
  size_t shm_size_;
  void * shm_mutex_;  // sem_t*
  std::string shm_name_;

  // Local cache for lock-free queries
  struct CachedEndpointInfo
  {
    rmw_endpoint_locality_t locality;
    uint64_t instance_id;
    EntityType entity_type;
  };
  mutable std::unordered_map<size_t, CachedEndpointInfo> local_cache_;
  mutable std::mutex cache_mutex_;
};

}  // namespace host_endpoint_manager

#endif  // HOST_ENDPOINT_MANAGER__HOST_ENDPOINT_MANAGER_HPP_
