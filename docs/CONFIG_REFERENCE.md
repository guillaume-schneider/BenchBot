---
icon: material/cog
---

# ⚙️ Configuration Reference

A comprehensive glossary of all configuration parameters available in `config_resolved.yaml`.

## 🧭 General Settings

| Parameter | Type | Default | Description |
| :--- | :--- | :--- | :--- |
| `run_name` | `string` | `RUN_001` | Unique identifier for the benchmark instance. |
| `output_dir` | `path` | `./results` | Directory where logs, bags, and metrics are saved. |
| `headless` | `bool` | `false` | If `true`, suppresses all GUI windows (for CI/CD). |

---

## 🤖 Robot Configuration

Parameters defining the robot's physical constraints and sensors.

??? example "Example `robot.yaml`"
    ```yaml
    robot:
      radius: 0.25
      max_speed: 0.5
      lidar:
        range: 12.0
        fov: 360
    ```

**`radius`**
:   *(float)* Radius of the robot's footprint in meters. Used for collision checking.

**`max_speed`**
:   *(float)* Maximum linear velocity in m/s.

**`lidar.range`**
:   *(float)* Maximum effective range of the laser scanner.

---

## 🗺️ SLAM Parameters

Common parameters tuned by the Auto-Tuner.

### GMapping

**`linearUpdate`**
:   *(float)* Process a scan each time the robot translates this far (meters).
    *   *Recommended:* `0.2` - `0.5`

**`angularUpdate`**
:   *(float)* Process a scan each time the robot rotates this far (radians).
    *   *Recommended:* `0.1` - `0.5`

**`particles`**
:   *(int)* Number of particles in the filter. Higher = better accuracy but more CPU.
    *   *Typical:* `30` (fast) to `100` (accurate).

### Cartographer

**`TRAJECTORY_BUILDER_2D.use_imu_data`**
:   *(bool)* Whether to use IMU data for scan matching. Critical for 3D SLAM, optional for 2D.

**`submaps.num_range_data`**
:   *(int)* Number of scans to insert before creating a new submap.

---

## 📉 Evaluation Config

Settings for the metrics engine.

| Parameter | Description |
| :--- | :--- |
| `align_trajectories` | Use Umeyama alignment to match estimated path to ground truth. |
| `eval_rate_hz` | Frequency of pose recording (default: 10Hz). |
| `save_images` | Generate PNGs of coverage and error maps. |
