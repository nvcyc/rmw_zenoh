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

#include "host_endpoint_manager/host_endpoint_manager.hpp"

#include <fcntl.h>
#include <semaphore.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

#include <atomic>
#include <cstring>
#include <functional>
#include <sstream>
#include <stdexcept>

#include "rcutils/logging_macros.h"

namespace host_endpoint_manager
{

// Default maximum number of endpoints
constexpr size_t MAX_ENDPOINTS = 512;

/// Shared memory layout structures
struct EndpointEntry
{
  uint64_t instance_id;
  uint8_t gid[RMW_GID_STORAGE_SIZE];
  EntityType entity_type;
  bool active;
  char topic_or_service_name[256];
};

struct EndpointRegistry
{
  uint32_t version;
  char hostname[256];
  uint32_t max_entries;
  uint32_t num_entries;
  EndpointEntry entries[MAX_ENDPOINTS];
};

// Compile-time size verification
static_assert(
  sizeof(EndpointEntry) % 8 == 0,
  "EndpointEntry must be 8-byte aligned for efficient array access");
static_assert(
  alignof(EndpointEntry) >= 8,
  "EndpointEntry must have at least 8-byte alignment");

// Static member initialization
std::mutex HostEndpointManager::instances_mutex_;
std::unordered_map<size_t, std::weak_ptr<HostEndpointManager>>
HostEndpointManager::instances_;

//==============================================================================
std::shared_ptr<HostEndpointManager>
HostEndpointManager::get_instance(size_t domain_id)
{
  std::lock_guard<std::mutex> lock(instances_mutex_);

  // Check if instance already exists for this domain
  auto it = instances_.find(domain_id);
  if (it != instances_.end()) {
    if (auto existing = it->second.lock()) {
      // Instance still alive, return it
      return existing;
    }
    // Instance was destroyed, remove stale weak_ptr
    instances_.erase(it);
  }

  // Create new instance (constructor is private)
  // Note: Can't use make_shared with private constructor
  auto instance = std::shared_ptr<HostEndpointManager>(
    new HostEndpointManager(domain_id)
  );

  // Store weak_ptr for future lookups
  instances_[domain_id] = instance;

  return instance;
}

//==============================================================================
HostEndpointManager::HostEndpointManager(size_t domain_id)
: domain_id_(domain_id),
  instance_id_(0),
  shm_fd_(-1),
  shm_ptr_(nullptr),
  shm_size_(0),
  shm_mutex_(nullptr)
{
  // Generate unique instance_id using atomic counter
  static std::atomic<uint64_t> next_id{1};
  const_cast<uint64_t &>(instance_id_) =
    next_id.fetch_add(1, std::memory_order_relaxed);

  // Get hostname
  char hostname_buf[256];
  if (gethostname(hostname_buf, sizeof(hostname_buf)) != 0) {
    throw std::runtime_error("Failed to get hostname");
  }
  hostname_ = hostname_buf;

  // Create SHM name based on domain and hostname hash
  uint32_t host_hash = hash_string(hostname_);
  shm_name_ = "/ros2_hem_d" + std::to_string(domain_id) +
    "_h" + std::to_string(host_hash);

  // Calculate SHM size
  shm_size_ = sizeof(EndpointRegistry);

  // Open/create POSIX shared memory
  shm_fd_ = shm_open(shm_name_.c_str(), O_CREAT | O_RDWR, 0666);
  if (shm_fd_ < 0) {
    throw std::runtime_error(
            "Failed to open shared memory: " + std::string(strerror(errno)));
  }

  // Set size
  if (ftruncate(shm_fd_, shm_size_) != 0) {
    close(shm_fd_);
    throw std::runtime_error(
            "Failed to resize shared memory: " + std::string(strerror(errno)));
  }

  // Map to process address space
  shm_ptr_ = mmap(nullptr, shm_size_, PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd_, 0);
  if (shm_ptr_ == MAP_FAILED) {
    close(shm_fd_);
    throw std::runtime_error(
            "Failed to map shared memory: " + std::string(strerror(errno)));
  }

  // Open named mutex (using semaphore as mutex)
  std::string mutex_name = shm_name_ + "_mtx";
  shm_mutex_ = sem_open(mutex_name.c_str(), O_CREAT, 0666, 1);
  if (shm_mutex_ == SEM_FAILED) {
    munmap(shm_ptr_, shm_size_);
    close(shm_fd_);
    throw std::runtime_error(
            "Failed to open mutex: " + std::string(strerror(errno)));
  }

  // Get pointer to registry
  auto * registry = static_cast<EndpointRegistry *>(shm_ptr_);

  // Initialize registry if first process
  sem_t * sem = static_cast<sem_t *>(shm_mutex_);
  sem_wait(sem);

  if (registry->version == 0) {
    // First process - initialize registry
    registry->version = 1;
    strncpy(registry->hostname, hostname_.c_str(), 255);
    registry->hostname[255] = '\0';
    registry->max_entries = MAX_ENDPOINTS;
    registry->num_entries = 0;

    // Initialize all entries as inactive
    for (size_t i = 0; i < MAX_ENDPOINTS; ++i) {
      registry->entries[i].active = false;
    }

    RCUTILS_LOG_INFO_NAMED(
      "host_endpoint_manager",
      "Initialized shared memory registry for domain %zu on host %s",
      domain_id, hostname_.c_str());
  } else {
    RCUTILS_LOG_DEBUG_NAMED(
      "host_endpoint_manager",
      "Attached to existing shared memory registry for domain %zu",
      domain_id);
  }

  sem_post(sem);
}

//==============================================================================
HostEndpointManager::~HostEndpointManager()
{
  if (shm_ptr_ != nullptr && shm_ptr_ != MAP_FAILED) {
    // Unregister all our endpoints
    auto * registry = static_cast<EndpointRegistry *>(shm_ptr_);
    sem_t * sem = static_cast<sem_t *>(shm_mutex_);

    if (sem != nullptr && sem != SEM_FAILED) {
      sem_wait(sem);

      for (size_t i = 0; i < registry->num_entries; ++i) {
        if (registry->entries[i].active &&
          registry->entries[i].instance_id == instance_id_)
        {
          registry->entries[i].active = false;
        }
      }

      sem_post(sem);

      // Close semaphore
      sem_close(sem);
    }

    // Unmap shared memory
    munmap(shm_ptr_, shm_size_);
  }

  if (shm_fd_ >= 0) {
    close(shm_fd_);
  }
}

//==============================================================================
bool HostEndpointManager::register_publisher(
  const rmw_gid_t & gid,
  const char * topic_name)
{
  return register_endpoint(gid, topic_name, EntityType::PUBLISHER);
}

//==============================================================================
bool HostEndpointManager::register_subscription(
  const rmw_gid_t & gid,
  const char * topic_name)
{
  return register_endpoint(gid, topic_name, EntityType::SUBSCRIPTION);
}

//==============================================================================
bool HostEndpointManager::register_service_client(
  const rmw_gid_t & gid,
  const char * service_name)
{
  return register_endpoint(gid, service_name, EntityType::SERVICE_CLIENT);
}

//==============================================================================
bool HostEndpointManager::register_service_server(
  const rmw_gid_t & gid,
  const char * service_name)
{
  return register_endpoint(gid, service_name, EntityType::SERVICE_SERVER);
}

//==============================================================================
bool HostEndpointManager::register_endpoint(
  const rmw_gid_t & gid,
  const char * name,
  EntityType type)
{
  auto * registry = static_cast<EndpointRegistry *>(shm_ptr_);
  sem_t * sem = static_cast<sem_t *>(shm_mutex_);

  // Lock SHM
  sem_wait(sem);

  // Find existing slot or allocate new one
  EndpointEntry * slot = nullptr;
  size_t slot_idx = 0;

  // First, check if this GID is already registered (update case)
  for (size_t i = 0; i < registry->num_entries; ++i) {
    if (memcmp(registry->entries[i].gid, gid.data, RMW_GID_STORAGE_SIZE) == 0) {
      slot = &registry->entries[i];
      slot_idx = i;
      break;
    }
  }

  // If not found, allocate new slot
  if (slot == nullptr) {
    if (registry->num_entries >= registry->max_entries) {
      sem_post(sem);
      RCUTILS_LOG_ERROR_NAMED(
        "host_endpoint_manager",
        "Shared memory registry full (max %u entries)",
        registry->max_entries);
      return false;
    }

    slot = &registry->entries[registry->num_entries];
    slot_idx = registry->num_entries;
    registry->num_entries++;
  }

  // Write to SHM
  slot->instance_id = instance_id_;
  memcpy(slot->gid, gid.data, RMW_GID_STORAGE_SIZE);
  slot->entity_type = type;
  slot->active = true;
  if (name != nullptr) {
    strncpy(slot->topic_or_service_name, name, 255);
    slot->topic_or_service_name[255] = '\0';
  } else {
    slot->topic_or_service_name[0] = '\0';
  }

  sem_post(sem);

  // Update local cache (we know it's intra-process for our own endpoints)
  size_t gid_hash = hash_gid(gid);
  {
    std::lock_guard<std::mutex> lock(cache_mutex_);
    local_cache_[gid_hash] = {
      EndpointLocality::INTRA_PROCESS,
      instance_id_,
      type
    };
  }

  RCUTILS_LOG_DEBUG_NAMED(
    "host_endpoint_manager",
    "Registered endpoint (type=%d) at slot %zu: %s",
    static_cast<int>(type), slot_idx, name ? name : "<unnamed>");

  return true;
}

//==============================================================================
bool HostEndpointManager::unregister_endpoint(const rmw_gid_t & gid)
{
  auto * registry = static_cast<EndpointRegistry *>(shm_ptr_);
  sem_t * sem = static_cast<sem_t *>(shm_mutex_);

  // Lock SHM
  sem_wait(sem);

  bool found = false;
  for (size_t i = 0; i < registry->num_entries; ++i) {
    if (registry->entries[i].active &&
      memcmp(registry->entries[i].gid, gid.data, RMW_GID_STORAGE_SIZE) == 0 &&
      registry->entries[i].instance_id == instance_id_)
    {
      registry->entries[i].active = false;
      found = true;
      break;
    }
  }

  sem_post(sem);

  // Remove from local cache
  if (found) {
    size_t gid_hash = hash_gid(gid);
    std::lock_guard<std::mutex> lock(cache_mutex_);
    local_cache_.erase(gid_hash);
  }

  return found;
}

//==============================================================================
LocalityInfo HostEndpointManager::query_endpoint_locality(const rmw_gid_t & gid) const
{
  size_t gid_hash = hash_gid(gid);

  // Lock-free lookup in local cache
  // Note: unordered_map reads are thread-safe without concurrent writes
  auto it = local_cache_.find(gid_hash);

  if (it == local_cache_.end()) {
    // NOT FOUND = not on this host (or not yet discovered)
    return {
      EndpointLocality::UNDEFINED,
      EntityType::PUBLISHER,  // placeholder
      0,
      false  // found = false
    };
  }

  // Found in cache = same-host endpoint
  return {
    it->second.locality,
    it->second.entity_type,
    it->second.instance_id,
    true  // found = true
  };
}

//==============================================================================
void HostEndpointManager::refresh_from_remote()
{
  auto * registry = static_cast<EndpointRegistry *>(shm_ptr_);
  sem_t * sem = static_cast<sem_t *>(shm_mutex_);

  // Lock SHM for reading
  sem_wait(sem);

  // Scan all active entries and build update map
  std::unordered_map<size_t, CachedEndpointInfo> updates;

  for (size_t i = 0; i < registry->num_entries; ++i) {
    if (!registry->entries[i].active) {
      continue;
    }

    const auto & entry = registry->entries[i];
    size_t gid_hash = hash_gid_raw(entry.gid);

    // Determine locality based on instance ID
    EndpointLocality locality =
      (entry.instance_id == instance_id_) ?
      EndpointLocality::INTRA_PROCESS :
      EndpointLocality::INTER_PROCESS_SAME_HOST;

    updates[gid_hash] = {
      locality,
      entry.instance_id,
      entry.entity_type
    };
  }

  sem_post(sem);

  // Batch update local cache
  {
    std::lock_guard<std::mutex> lock(cache_mutex_);
    for (const auto & [hash, info] : updates) {
      local_cache_[hash] = info;
    }
  }

  RCUTILS_LOG_DEBUG_NAMED(
    "host_endpoint_manager",
    "Refreshed cache with %zu endpoints",
    updates.size());
}

//==============================================================================
void HostEndpointManager::refresh_endpoints(const std::vector<rmw_gid_t> & gids)
{
  // For now, just do a full refresh
  // TODO(maintainer): Optimize to only refresh specific GIDs
  (void)gids;
  refresh_from_remote();
}

//==============================================================================
uint64_t HostEndpointManager::get_instance_id() const
{
  return instance_id_;
}

//==============================================================================
size_t HostEndpointManager::get_domain_id() const
{
  return domain_id_;
}

//==============================================================================
std::string HostEndpointManager::get_hostname() const
{
  return hostname_;
}

//==============================================================================
Stats HostEndpointManager::get_stats() const
{
  auto * registry = static_cast<EndpointRegistry *>(shm_ptr_);

  Stats stats;
  stats.local_cache_size = local_cache_.size();
  stats.shm_entries = registry->num_entries;
  stats.shm_capacity = registry->max_entries;

  return stats;
}

//==============================================================================
size_t HostEndpointManager::hash_gid(const rmw_gid_t & gid)
{
  return hash_gid_raw(gid.data);
}

//==============================================================================
size_t HostEndpointManager::hash_gid_raw(const uint8_t * gid)
{
  // Simple hash of GID bytes
  std::hash<std::string> hasher;
  std::string gid_str(reinterpret_cast<const char *>(gid), RMW_GID_STORAGE_SIZE);
  return hasher(gid_str);
}

//==============================================================================
uint32_t HostEndpointManager::hash_string(const std::string & str)
{
  // Simple 32-bit hash for hostname
  uint32_t hash = 0;
  for (char c : str) {
    hash = hash * 31 + static_cast<uint32_t>(c);
  }
  return hash;
}

}  // namespace host_endpoint_manager
