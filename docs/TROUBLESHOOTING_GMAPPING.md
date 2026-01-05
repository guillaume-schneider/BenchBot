# Troubleshooting Exploration: GMapping & Nav2 Integration

This document details the critical issues encountered and resolved to enable robust autonomous exploration using **GMapping**, **Nav2**, and **explore_lite** within the Orchestrator.

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

## 2. Parameter Tuning for Stability

### Synchronizing Time & TF
**Issue:**
Nav2 uses `use_sim_time: true`. If GMapping runs in wall-time, timestamps diverge by ~50+ years, causing total system failure.

**Resolution:**
- Created `configs/params/gmapping_params.yaml` to enforce `use_sim_time: true` explicitly.
- Modified `gmapping.yaml` launch command to explicitly inject `-p use_sim_time:=true` into the node args, as the wrapper script isolates the environment.
- **Tuned TF Rate**: Set `transform_publish_period` to **0.02s (50Hz)** to match SLAM Toolbox's high-frequency standard, ensuring smooth costmap updates.

### Robot Footprint & Safety
**Issue:**
The robot would "spin forever" or get stuck in open space.
**Root Cause:**
- `robot_radius` was 0.15m (too small for Waffle).
- `inflation_radius` was 0.20m (too small).
The robot tried to navigate through gaps it physically couldn't fit into, then got stuck in a recovery loop.

**Resolution (`nav2_params.yaml`):**
- `robot_radius` -> **0.22m** (Waffle physical size).
- `inflation_radius` -> **0.55m** (Standard safety margin).
- `cost_scaling_factor` -> **3.0** (Smoother gradient).

### Exploration Range
**Issue:**
`explore_lite` failed to find frontiers ("Exploration stopped") immediately.
**Root Cause:**
GMapping's `maxUrange` (usable range) was defaulted to 80m or clamped incorrectly. The robot didn't "know" space was free.
**Resolution:**
- Set `maxUrange: 20.0` to maximize the "known free" area used for planning.
- Decreased `min_frontier_size` to **0.05m** to allow detecting small initial frontiers.

### Exploration Stopping Prematurely "After One Frontier"
**Issue:**
The robot would map one section/room and then stop, even though unexplored areas remained nearby.
**Root Cause:**
`potential_scale` was set to **3.0** (too high). This parameter applies a "penalty" proportional to the distance to the frontier. A high value makes distant frontiers have a negative utility (score), causing the planner to reject them and assume no valid goals exist.
**Resolution:**
- Set `potential_scale` to **0.001** (default recommended value). This drastically reduces the distance penalty, ensuring even far-away frontiers are considered reliable targets.
- Increased `planner_frequency` to **1.0 Hz** (from 0.33) to reduce the idle "thinking" time between goals.

---

## 3. Workflow Integration Fixes

### Costmap Topic Mismatch
**Issue:**
`explore_lite` stuck waiting: `[INFO] Waiting for costmap to become available...`
**Root Cause:**
Default topic is `costmap`. Nav2 outputs `/global_costmap/costmap`. GMapping outputs `/map`.
**Resolution:**
- Created `configs/params/explore_params.yaml`.
- Set `costmap_topic: /global_costmap/costmap` to use the **inflated** Nav2 map (safer) or `/map` (raw). We opted for a configuration that ensures connection.

### Launch Argument Crash
**Issue:**
`UnknownROSArgsError: found unknown ROS arguments: 'use_rviz:=True'`.
**Root Cause:**
The GUI orchestrator injects `use_rviz:=True` into *every* command. `ros2 run` (used for explore_lite) crashes if it receives launch-style arguments.
**Resolution:**
Modified `explore_wrapper.sh` to filter out `use_rviz*` arguments before executing the node.
