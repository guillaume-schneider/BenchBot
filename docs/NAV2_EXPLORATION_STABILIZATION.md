# Nav2 & Exploration Stabilization Documentation

This document records the critical fixes applied to resolve issues with robot navigation, autonomous exploration, and 3D visualization stability.

## 1. Nav2 Framework Activation Fix

### Issue
Nav2 nodes (`controller_server`, `planner_server`, etc.) remained in the `inactive` state and timed out during transitions.
*   **Root Cause:** The configuration in `nav2_params.yaml` used `base_link` as the `robot_base_frame`. However, the TurtleBot3 simulation model uses `base_footprint` as its root link.
*   **Symptom:** Logs showed `failed to send response to /planner_server/change_state (timeout)` and TF lookup failures.

### Resolution
Updated `configs/params/nav2_params.yaml`:
*   Changed all instances of `robot_base_frame: base_link` to `robot_base_frame: base_footprint`.
*   **Result:** Nav2 nodes now transition to the `active` state immediately upon startup.

## 2. Autonomous Exploration Connectivity

### Issue
The `explore_lite` node was unable to command robot movement.
*   **Root Cause:** `explore_lite` attempts to connect to a ROS 1 style action server named `move_base`. Modern Nav2 uses `/navigate_to_pose`.
*   **Symptom:** Logs appeared as `Waiting to connect to move_base nav2 server` indefinitely.

### Resolution
Updated `tools/launch/explore_with_qos.launch.py`:
*   Added a ROS 2 remapping: `('move_base', '/navigate_to_pose')`.
*   **Result:** Explorer successfully connects to Nav2 and issues navigation goals.

## 3. 3D Visualizer Stability

### Issue
The 3D trajectory in the GUI appeared as a "flying" mesh or a "fan" pattern (toile d'araignée).
*   **Root Causes:**
    1.  Micro-vibrations in the simulation caused small Z-axis fluctuations, which accumulated in the 3D trace.
    2.  Simulation "spikes" (huge jumps in odometry) during the first few milliseconds of Gazebo startup.
    3.  Recording points at origin (0,0,0) before the robot was spawned.

### Resolution
Updated `gui/pages/visualizer.py`:
*   **Z-Projection:** Forced $Z=0$ for all trajectory points and robot axes.
*   **Spike Filter:** Added a distance check—if the robot moves > 2.0 meters in a single update, the point is ignored.
*   **Origin Filter:** Ignored poses at exactly (0,0,0) during the first few seconds.
*   **Yaw Only:** Restricted robot rotation to the horizontal plane (Yaw).

## 4. Environment Cleanup (Ghost Processes)

### Issue
"TF_OLD_DATA" warnings and "frozen" odometry occurred due to leftover processes from previous crashed runs.

### Resolution
Updated `runner/orchestrator.py`:
*   Expanded the `pkill` target list in the preventive cleanup phase.
*   Added: `slam_gmapping`, `sync_slam_toolbox_node`, `explore_node`, `spawn_entity`, and specific Nav2 manager nodes.
*   **Result:** Every run now starts on a perfectly clean slate.
