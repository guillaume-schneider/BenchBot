# Troubleshooting Exploration: GMapping, Cartographer & Nav2 Integration

This document details the critical issues encountered and resolved to enable robust autonomous exploration using **GMapping** and **Cartographer** with **Nav2** and **explore_lite**.

## 1. GMapping ROS 2 Port (slam_gmapping) Fixes

### Critical Bug: "Lookup would require extrapolation into the past"
**Symptoms:**
- The SLAM node runs, but **Nav2** (global/local costmaps) continuously spam errors: `TF_OLD_DATA` or `Lookup would require extrapolation into the past`.
- The robot cannot localize or plan because the map frame is "too old" relative to `odom`.

**Root Cause:**
In the ported `slam_gmapping.cpp` source code, the TF expiration time calculation was flawed:
```cpp
// BAD CODE
rclcpp::Time tf_expiration = get_clock()->now() + rclcpp::Duration(static_cast<int32_t>(tf_delay_), 0);
// tf_delay_ is a double (0.05). casting to int32_t makes it 0.
// Result: Expiration = NOW + 0s.
```
Any network or processing latency (> 0ms) caused the transform to arrive "expired" at the consumer (Nav2).

**Resolution:**
The source code was patched to correctly construct the duration from seconds:
```cpp
rclcpp::Duration delay_duration = rclcpp::Duration::from_seconds(tf_delay_);
rclcpp::Time tf_expiration = get_clock()->now() + delay_duration;
```
This ensures the TF is valid for the specified duration (e.g., 0.02s wait + buffer).

### Parameter Loading Failure
**Symptoms:**
- Changing parameters like `maxUrange` or `delta` in YAML had **no effect**.
- The `transform_publish_period` was stuck at a hardcoded default, causing sync issues.

**Root Cause:**
The `init()` function of the node did not call `declare_parameter` or `get_parameter` for any of the GMapping settings. It simply assigned hardcoded defaults.

**Resolution:**
We implemented comprehensive parameter declaration and retrieval in `slam_gmapping.cpp`, allowing dynamic configuration via YAML/Launch.

---

## 2. Cartographer Configuration Fixes

### Issue: "Could not find nearby clear cell" (Stuck at start)
**Symptoms:**
- `explore_lite` starts but immediately warns: `[FrontierSearch] Could not find nearby clear cell to start search`.
- The robot refuses to move.

**Root Cause:**
Cartographer, by default, sets the starting cell under the robot as "Unknown" (grey) because the Lidar cannot see "inside" the robot itself (`min_range > 0`). `explore_lite` requires the robot to be on a "Free" (white) cell to start.

**Resolution (`cartographer_turtlebot3_2d.lua`):**
1.  **Zero Minimum Range**:
    ```lua
    TRAJECTORY_BUILDER_2D.min_range = 0.0
    ```
    This forces Cartographer to mark space as free starting directly from the sensor origin.
2.  **Instant Update**:
    ```lua
    TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 1
    ```
    Typically Cartographer accumulates scans. Setting this to 1 forces an immediate map update for every scan, clearing the start area instantly.

### Issue: Start Timing & Kickstart
**Symptoms:**
- Even with the config fix, the robot sometimes stayed stuck if `explore_lite` started too early.

**Resolution (`test_cartographer.yaml`):**
- **Kickstart Delay**: Set `kickstart.sh` delay to **5.0s**. This ensures Gazebo is fully ready before the robot performs its initialization spin.
- **Explore Delay**: Set `explore` delay to **20.0s**. This ensures the kickstart maneuver (which takes ~5s + buffers) is COMPLETELY finished and the map is cleared before exploration attempts to plan.

---

## 3. Navigation & Path Planning (Nav2)

### Issue: Robot Refusing to Pass Through Doors
**Symptoms:**
- The robot would verify a frontier exists through a door, plan a global path, but then abort or recover indefinitely when trying to execute it.
- RViz showed the costmap inflation merging in the doorway, making it appear "lethal" or high-cost.

**Root Cause:**
The standard `robot_radius` (0.22m) combined with a conservative `inflation_radius` (0.35m) essentially "closed" narrow passages in the Costmap.

**Resolution (`nav2_params.yaml`):**
1.  **Rectangular Footprint**:
    Replaced the circular radius with the exact TurtleBot3 Waffle footprint:
    ```yaml
    footprint: "[ [0.14, 0.15], [0.14, -0.15], [-0.14, -0.15], [-0.14, 0.15] ]"
    ```
    This allows the planner to orient the robot to fit through gaps that are narrower than its diagonal diameter.

2.  **Surgical Inflation Tuning** (Local & Global):
    - **`inflation_radius: 0.20`**: Inflation stops 20cm from walls. Since footprint is ~15cm half-width + safety, this prevents "phantom walls" in doorways.
    - **`cost_scaling_factor: 10.0`**: sharp decay. The cost drops to near-zero immediately after the lethal radius, encouraging the robot to pass close to walls if necessary.

---

## 4. Explore Lite Tuning

### Issue: Stopping After One Room
**Symptoms:**
- The robot maps one area and declares "All frontiers traversed" while half the map is empty.

**Root Cause:**
The `potential_scale` parameter was default or set high (3.0). This adds a cost proportional to distance. For a text-book exploration, distant frontiers became "too expensive" compared to staying put.

**Resolution (`explore_params.yaml`):**
- **`potential_scale: 0.001`**: Drastically reduced distance penalty.
- **`min_frontier_size: 0.2`**: Increased to ignore noise/small gaps in walls (ghost frontiers).
- **`costmap_topic: /global_costmap/costmap`**: Switched from `/map` to Nav2's costmap. This is CRITICAL because Nav2's costmap automatically overwrites the robot's footprint as "Free Space", solving any remaining "stuck at start" issues.

---

## Summary of Success Configuration
- **GMapping**: Patched C++, tuned TF rate (50Hz), Parameter loading.
- **Cartographer**: `min_range = 0.0`, `accumulated_range = 1`, Delayed start (20s).
- **Nav2**: Rectangular Footprint, `inflation_radius = 0.20`, `scaling = 10.0`.
- **Explore**: Uses Global Costmap, `potential_scale = 0.001`.

---

## 5. Explorer Startup Delays \u0026 TF Synchronization

### Issue: Explorer Takes Long Time to Start
**Symptoms:**
- The `explore_lite` node starts but waits 10-20 seconds before beginning exploration.
- Logs show repeated `TF_OLD_DATA` warnings and `Timed out waiting for transform` errors.
- Example error: `Lookup would require extrapolation into the past. Requested time 5.683000 but the earliest data is at time 21.500000`

**Root Cause:**
The explorer starts too early, before Nav2 has fully initialized and published stable TF transformations. The explorer requires:
1. The `base_link` → `map` transformation to be available
2. The `/map` topic (costmap) to be published
3. The Nav2 action server to be ready

**Resolution:**
Add a startup delay to the explorer process using the `delay_s` parameter in the dataset configuration:

```yaml
processes:
  - name: "nav2_stack"
    cmd: [...]
    
  - name: "explore"
    delay_s: 10.0  # Wait for Nav2 to fully initialize
    cmd: [...]
```

**Recommended Delays:**
- **Nav2 Stack**: 0s (starts first)
- **Explorer**: 5s (waits for Nav2 - not too long to avoid TF desync)
- **Kickstart Script**: 3-5s (if used)

**Important Note on Delays:**
- Delays that are too long (>10s) can cause TF desynchronization issues where the explorer tries to access transforms from the past
- The `transform_tolerance` parameter in `explore_params.yaml` has been increased to 30.0s to handle pause/resume cycles
- The orchestrator now waits 2s after starting all processes before pausing exploration to ensure proper initialization

**How It Works:**
The orchestrator now supports the `delay_s` parameter for each process. When specified, it will sleep for that duration before starting the process, allowing previous processes to initialize fully.

**Benefits:**
- Eliminates TF synchronization errors
- Reduces startup warnings in logs
- Ensures stable exploration from the start
- No wasted time waiting for timeouts
