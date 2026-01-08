# 🎯 Multi-SLAM Benchmarking Guide

## 📋 Available SLAMs

Your orchestrator currently supports **4 SLAM algorithms**:

### 1. **SLAM Toolbox (Sync Mode)**
- **ID**: `slam_toolbox_sync`
- **Type**: Graph-based SLAM
- **Advantages**: State-of-the-art loop closure, very robust.
- **Config**: `configs/slams/slam_toolbox_sync.yaml`

### 2. **Cartographer 2D**
- **ID**: `cartographer_2d`
- **Type**: Submap Matching SLAM
- **Advantages**: High precision, optimized for large environments.
- **Config**: `configs/slams/cartographer_2d.yaml`

### 3. **GMapping (Native Path)**
- **ID**: `gmapping`
- **Type**: Particle filter SLAM
- **Notes**: Fixed & Patched in `deps/gmapping_ws` to support parameters and ROS 2 Humble.
- **Config**: `configs/slams/gmapping.yaml`

### 4. **External (Passive)**
- **ID**: `external`
- **Type**: Passive observer
- **Usage**: When SLAM is already running externally or as part of the scenario dataset.
- **Config**: `configs/slams/external.yaml`

### 5. **NoOp (Baseline)**
- **ID**: `noop`
- **Type**: Odometry-only (No SLAM)
- **Usage**: Reference point to measure drift without correction.

---

## 🚀 How to Test Different SLAMs

### Option 1: Via GUI (Recommended)

```bash
cd ~/Projects/slam_bench_orchestrator
python3 gui/main.py
```

1. In the **Dashboard**, select **`slam_comparison.yaml`**
2. Click **Run**
3. The orchestrator will launch **3 benchmarks** automatically:
   - Run 1: NoOp (baseline)
   - Run 2: SLAM Toolbox
   - Run 3: Cartographer
4. The results are displayed automatically!

### Option 2: Via CLI

```bash
cd ~/Projects/slam_bench_orchestrator
# Source ROS 2
source /opt/ros/humble/setup.bash

# Launch the matrix
python3 runner/run_matrix.py configs/matrices/slam_comparison.yaml
```

---

## 📊 Comparing Results

### Metrics Collected for Each SLAM

- **Coverage**: Discovery percentage of explorable area.
- **IoU**: Accuracy of generated map vs Ground Truth.
- **ATE**: Localization precision (RMSE).
- **Duration**: Actual wall clock time for the run.
- **CPU/Memory**: Peak system resources consumed (Max CPU%, Max RAM MB).
- **Path Length**: Total distance traveled.

### Visualizing the Comparison

In the GUI:
1. Go to **Details** (click on your matrix card)
2. **Results** Tab: Comparative table
3. Click on each run to see detailed metrics
4. Visually compare generated maps

---

## 🔧 Creating Your Own Matrix

### Example: Testing Only 2 SLAMs

```yaml
# configs/matrices/my_test.yaml
name: "My SLAM Test"

datasets:
  - include: "configs/datasets/tb3_sim_explore_modeA.yaml"

slams:
  - id: slam_toolbox_sync
    profile: configs/slams/slam_toolbox_sync.yaml
  - id: cartographer_2d
    profile: configs/slams/cartographer_2d.yaml

matrix:
  include:
    - dataset: tb3_sim_explore_modeA
      slams: [slam_toolbox_sync, cartographer_2d]
      seeds: [0]
      repeats: 1
```

### Example: Testing with Multiple Seeds (Robustness)

```yaml
matrix:
  include:
    - dataset: tb3_sim_explore_modeA
      slams: [slam_toolbox_sync]
      seeds: [0, 1, 2, 3, 4]  # 5 different runs
      repeats: 1
```

This will generate **5 runs** with different random initializations.

---

## 📝 Expected Results

### Results Structure

```
results/runs/
├── 2026-01-04_XX-XX-XX__tb3_sim_explore_modeA__noop__seed0__r0/
│   ├── bags/       # Recorded Rosbag
│   ├── logs/       # Logs for each process
│   └── metrics.json # Calculated metrics
├── 2026-01-04_XX-XX-XX__tb3_sim_explore_modeA__slam_toolbox_sync__seed0__r0/
│   └── ...
└── 2026-01-04_XX-XX-XX__tb3_sim_explore_modeA__cartographer_2d__seed0__r0/
    └── ...
```

### Comparison Example

| SLAM | Coverage | IoU | ATE | Runtime |
|------|----------|-----|-----|---------|
| NoOp | 45% | 0.35 | 2.5m | 90s |
| SLAM Toolbox | **78%** | **0.82** | **0.3m** | 95s |
| Cartographer | 72% | 0.79 | 0.4m | 110s |

**Winner**: SLAM Toolbox (better coverage and precision)

---

## 🐛 Troubleshooting

### Cartographer Does Not Launch

**Check** that Cartographer is installed:
```bash
ros2 pkg list | grep cartographer
```

**If missing**, install:
```bash
sudo apt install ros-humble-cartographer-ros
```

### GMapping Does Not Work

GMapping does not have an official ROS 2 port. Use:
- SLAM Toolbox (better)
- Cartographer (alternative)

### Error "Failed to resolve dependencies"

A SLAM config refers to a missing file. **Check**:
```bash
cat configs/slams/cartographer_2d.yaml
# Look at paths in 'configuration_directory'
```

**Adapt** the paths to your system.

---

## 🎯 Recommendations

### Getting Started
1. **Test** first with `noop` (baseline)
2. **Then** `slam_toolbox_sync` (most robust)
3. **Compare** with your objective

### For Performance
- **SLAM Toolbox**: Best speed/accuracy balance
- **Cartographer**: More accurate on large environments
- **NoOp**: Fastest (no SLAM)

### For Research
- **Multi-seeds**: Test robustness
- **Multi-datasets**: Test generalization
- **Multi-slams**: Comparative benchmarking

---

## 📚 Resources

- **SLAM Toolbox**: https://github.com/SteveMacenski/slam_toolbox
- **Cartographer**: https://github.com/cartographer-project/cartographer
- **ROS 2 SLAM**: https://github.com/ros-planning/navigation2

---

## 🎉 Next Level

### Adding a New SLAM

1. Create `configs/slams/my_slam.yaml`
2. Define the launch command
3. Add it to your matrix
4. Launch!

**Example**: Adding Hector SLAM:

```yaml
# configs/slams/hector_slam.yaml
schema_version: 1
id: "hector_slam"
display_name: "Hector SLAM"

launch:
  cmd: ["ros2", "launch", "hector_slam", "hector_slam.launch.py"]
  use_sim_time: true

io_contract:
  map_topic: "/map"
  scan_topic: "/scan"

probes:
  ready:
    - type: topic_publish
      topic: /map
      timeout_s: 60
```

Then add it to your matrix!

---

**Happy Benchmarking!** 🚀
