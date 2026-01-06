# Host Endpoint Manager

RMW-agnostic host-local endpoint locality manager for transport optimization in ROS 2.

## Overview

The Host Endpoint Manager tracks RMW endpoint locality (publishers, subscriptions, service clients, service servers) to enable transport optimization. It uses a singleton-per-domain pattern with POSIX shared memory for inter-process coordination and provides lock-free locality queries.

## Key Features

- **Singleton Per Domain**: One manager instance per (process, domain_id) pair ensures correct intra-process detection across multiple RMW contexts
- **RMW-Agnostic**: Works with any RMW implementation (rmw_zenoh, rmw_fastrtps, etc.) - only depends on `rmw` and `rcutils`
- **Lock-Free Queries**: O(1) endpoint locality lookup with no IPC overhead on the critical path
- **Same-Host Only Tracking**: Focuses on local endpoints where optimization is possible
- **Hostname-Based Isolation**: Container-safe via hostname-based shared memory naming
- **Thread-Safe**: All operations safe for concurrent access from multiple threads

## Architecture

```
┌─────────────────────────────────────────────────────────┐
│ Process 1 (PID 1234)                                    │
│ ┌─────────────────────────────────────────────────────┐ │
│ │ Singleton Instance (domain 0, instance_id 1001)     │ │
│ │ ┌──────────────┐  ┌──────────────┐                 │ │
│ │ │ RMW Context A│  │ RMW Context B│                 │ │
│ │ └──────┬───────┘  └──────┬───────┘                 │ │
│ │        └──────────────────┘                         │ │
│ │               │                                      │ │
│ │        ┌──────▼──────┐                              │ │
│ │        │ Local Cache │ (lock-free queries)          │ │
│ │        └─────────────┘                              │ │
│ └───────────────┬─────────────────────────────────────┘ │
└─────────────────┼───────────────────────────────────────┘
                  │ write (SHM mutex)
                  ▼
    ┌─────────────────────────────────┐
    │ POSIX Shared Memory              │
    │ /ros2_hem_d0_h<hostname_hash>   │
    │ ┌─────────────────────────────┐ │
    │ │ EndpointRegistry            │ │
    │ │ - hostname: "robot1"        │ │
    │ │ - entries[512]              │ │
    │ └─────────────────────────────┘ │
    └──────────┬──────────────────────┘
               │ read (SHM mutex)
┌──────────────▼──────────────────────────────────────────┐
│ Process 2 (PID 5678)                                    │
│ ┌─────────────────────────────────────────────────────┐ │
│ │ Singleton Instance (domain 0, instance_id 1002)     │ │
│ │ ┌──────────────┐                                    │ │
│ │ │ RMW Context C│                                    │ │
│ │ └──────┬───────┘                                    │ │
│ │ ┌──────▼──────┐                                     │ │
│ │ │ Local Cache │                                     │ │
│ │ └─────────────┘                                     │ │
│ └─────────────────────────────────────────────────────┘ │
└─────────────────────────────────────────────────────────┘
```

## Locality Detection

The manager classifies endpoints into three categories:

1. **INTRA_PROCESS**: Same `instance_id` (same process + domain)
   - Multiple RMW contexts in the same process share the same singleton
   - Enables zero-copy optimization
   
2. **INTER_PROCESS_SAME_HOST**: Different `instance_id` but found in SHM
   - Different process on the same host
   - Enables shared memory transport optimization
   
3. **UNDEFINED** (treated as INTER_HOST): Not found in cache
   - Endpoint not registered on this host
   - RMW should use network transport

## Usage

### RMW Integration

#### 1. Get Singleton Instance

```cpp
#include "host_endpoint_manager/host_endpoint_manager.hpp"

// In rmw_init() or context initialization
auto endpoint_manager = 
  host_endpoint_manager::HostEndpointManager::get_instance(domain_id);

// Store in context for later use
context_impl->endpoint_manager = endpoint_manager;
```

#### 2. Register Endpoints

```cpp
// When creating a publisher
rmw_gid_t pub_gid;
rmw_get_gid_for_publisher(publisher, &pub_gid);
endpoint_manager->register_publisher(pub_gid, topic_name);

// When creating a subscription
rmw_gid_t sub_gid;
// (extract GID from subscription data structure)
endpoint_manager->register_subscription(sub_gid, topic_name);

// When creating a service client
rmw_gid_t client_gid;
rmw_get_gid_for_client(client, &client_gid);
endpoint_manager->register_service_client(client_gid, service_name);

// When creating a service server
rmw_gid_t server_gid;
// (extract GID from service data structure)
endpoint_manager->register_service_server(server_gid, service_name);
```

#### 3. Query Locality (Lock-Free)

```cpp
// In rmw_publish() or message send path
auto info = endpoint_manager->query_endpoint_locality(subscriber_gid);

if (!info.found) {
  // Endpoint not on this host → use network transport
  serialize_and_send_over_network(msg);
}
else if (info.locality == RMW_ENDPOINT_LOCALITY_INTRA_PROCESS) {
  // Same process → zero-copy optimization
  use_zero_copy_transfer(msg);
}
else if (info.locality == RMW_ENDPOINT_LOCALITY_INTER_PROCESS_SAME_HOST) {
  // Same host, different process → shared memory optimization
  use_shm_transport(msg);
}
```

#### 4. Refresh Cache

```cpp
// In graph event callback when discovering new same-host endpoints
void on_remote_endpoint_discovered() {
  // Check if endpoint is on same host (implementation-specific)
  if (is_same_host_endpoint(entity)) {
    endpoint_manager->refresh_from_remote();
  }
}
```

#### 5. Unregister Endpoints

```cpp
// When destroying an endpoint
endpoint_manager->unregister_endpoint(gid);
```

## Multi-Context Scenario

The singleton pattern ensures correct intra-process detection even with multiple RMW contexts:

```cpp
// Process with component container (multiple contexts)

// Context 1 initialization
auto mgr1 = HostEndpointManager::get_instance(0);  // Creates singleton, id=1001

// Context 2 initialization (same process, same domain)
auto mgr2 = HostEndpointManager::get_instance(0);  // Gets SAME singleton, id=1001

// mgr1 == mgr2 (same pointer)
// Both contexts share the same instance_id

// Publisher in context 1
mgr1->register_publisher(pub_gid, "topic");  // Stored with instance_id=1001

// Subscriber in context 2
mgr2->register_subscription(sub_gid, "topic");  // Stored with instance_id=1001

// Query from context 1 about context 2's subscriber
auto info = mgr1->query_endpoint_locality(sub_gid);
// Returns: locality=INTRA_PROCESS ✓ (correctly detected!)
```

## Thread Safety

- **Singleton Creation**: Protected by `instances_mutex_` - safe for concurrent `get_instance()` calls
- **Registration**: Protected by POSIX named semaphore (SHM mutex) - safe across processes
- **Queries**: Lock-free read-only access to local cache - no synchronization overhead
- **Refresh**: Protected by SHM mutex for reads, cache mutex for updates
- **Concurrent Access**: Multiple threads can safely call all methods

## Limitations

1. **Same-Host Only**: Does not track inter-host endpoints (they return `found=false`)
2. **Fixed Capacity**: Default 512 endpoints per domain/host (configurable via `MAX_ENDPOINTS`)
3. **Container Isolation**: Different containers must have different hostnames
   - Containers with the same hostname will share SHM (usually incorrect)
   - Most container runtimes assign unique hostnames by default
4. **No Refresh Optimization**: `refresh_endpoints(gids)` currently does full refresh

## Performance Characteristics

- **Query**: O(1) hash map lookup, lock-free, ~10-50 nanoseconds typical
- **Registration**: O(n) scan for existing entry + O(1) update, ~1-10 microseconds typical
- **Refresh**: O(n) where n is number of entries, ~10-100 microseconds for 100 entries
- **Memory**: ~138 KB shared memory for 512 entries (270 bytes per entry)

## Debugging

### Check Statistics

```cpp
auto stats = endpoint_manager->get_stats();
std::cout << "Local cache size: " << stats.local_cache_size << std::endl;
std::cout << "SHM entries: " << stats.shm_entries << " / " 
          << stats.shm_capacity << std::endl;
```

### Inspect Shared Memory

```bash
# List shared memory objects
ls -lh /dev/shm/ros2_hem_*

# View with hexdump (first 512 bytes)
xxd /dev/shm/ros2_hem_d0_h<hash> | head -n 32
```

### Enable Debug Logging

```bash
export RCUTILS_CONSOLE_OUTPUT_FORMAT="[{severity}] [{name}]: {message}"
export RCUTILS_LOGGING_USE_STDOUT=1
export RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED=1

# Run your ROS 2 application
ros2 run <package> <node> --ros-args --log-level host_endpoint_manager:=debug
```

## Dependencies

- **rmw**: For `rmw_gid_t`, `rmw_endpoint_locality_t` types
- **rcutils**: For logging macros
- **POSIX**: For `shm_open`, `mmap`, `sem_open` (librt)
- **C++17**: For standard library features

## Building

```bash
cd /path/to/ros_ws
colcon build --packages-select host_endpoint_manager
```
