# Robot Hardware & Sensor Manager 🛰️⚙️

The Robot Manager allows you to test the **robustness** and **resilience** of SLAM algorithms by simulating real-world hardware limitations without modifying your simulation world or robot models.

## 🛠️ Configuration Options

### 1. LIDAR Sensor Emulation
*   **Max Range**: Limit how far the LIDAR can see (e.g., 3.0m instead of 10.0m). This tests SLAM performance in wide-open spaces where few landmarks are in range.
*   **Gaussian Noise**: Add "Salt and Pepper" noise to the scan data. This simulates low-quality sensors or interference.

### 2. Chassis & Actuators
*   **Speed Scaling**: Throttle the robot's maximum velocity (1% to 200%). Testing at higher speeds helps identify SLAM algorithms that fail under high-frequency movement.

## 🚀 Quick Presets

We provide several built-in scenarios to quickly test your SLAM:
- **Bad LIDAR**: Very short range (3m) and significant noise.
- **Weak Motors**: Forces the robot to move slowly, testing "patience" and drift over time.
- **Extreme Stress**: Very noisy, very short range – only the most robust algorithms will survive.

## 🔧 How it Works (Technical Details)

When "Hardware Degradation" is enabled:
1.  The orchestrator starts the `tools/sensor_degrader.py` node.
2.  The simulator (Gazebo/O3DE) is instructed via remapping to publish to `/scan_raw` and `/cmd_vel_raw`.
3.  The Degrader node intercepts these "raw" topics, applies your noise/limits, and republishes them as `/scan` and `/cmd_vel`.
4.  The SLAM and Navigation stack subscribe to the "dirty" data, unaware of the interception.

> **Note**: This feature is non-destructive. Disabling it and saving will immediately return the robot to "Perfect Sensor" mode for the next run.
