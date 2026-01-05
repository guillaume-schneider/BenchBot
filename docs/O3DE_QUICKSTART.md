# Quick Start: Using O3DE with SLAM Bench Orchestrator

## 🎯 Prerequisites

1. **O3DE Installed** (via GUI: Tools → Simulators → Install O3DE)
2. **Existing Gazebo world** (SDF format)

## 🚀 3-Step Workflow

### Step 1: Create O3DE Dataset

```yaml
# configs/datasets/my_o3de_test.yaml
name: "My O3DE Test"
id: "my_o3de_test"

simulator: "o3de"  # ← This is the magic line!

scenario:
  world_model: "worlds/model.sdf"  # Your existing Gazebo world
  
  processes:
    - name: "nav2_stack"
      cmd: [ros2, launch, nav2_bringup, ...]
    # ... other processes
```

### Step 2: Create Matrix (or use existing)

```yaml
# configs/matrices/test_o3de.yaml
datasets:
  - include: "configs/datasets/my_o3de_test.yaml"

slams:
  - include: "configs/slams/noop.yaml"

matrix:
  include:
    - datasets: ["my_o3de_test"]
      slams: ["noop"]
      seeds: [0]
      repeats: 1
```

### Step 3: Run!

**Via GUI:**
```bash
python3 gui/main.py
# Dashboard → Select "test_o3de.yaml" → RUN
```

**Via CLI:**
```bash
python3 runner/run_one.py configs/matrices/test_o3de.yaml
```

## 📊 What Happens Automatically

1. **Orchestrator detects** `simulator: o3de`
2. **Verifies** O3DE is installed
3. **Converts** your SDF world to O3DE project (cached!)
   - Example: `worlds/model.sdf` → `~/.slam_bench/o3de/projects/model_o3de_project`
4. **Launches** O3DE with the project
5. **Launches** Nav2, Explore, etc. (same as Gazebo)
6. **Records** rosbag (identical to Gazebo)
7. **Computes** metrics (Coverage, IoU, ATE)

## 🔍 Monitoring

**Check logs:**
```bash
# During run
tail -f results/runs/LATEST/logs/o3de_sim.log

# After run
cat results/runs/2026-01-04_18-30-00__my_o3de_test__noop__seed0__r0/logs/o3de_sim.log
```

**Verify O3DE process:**
```bash
ps aux | grep Editor
```

## 🆚 Comparing Gazebo vs O3DE

Use the provided comparison matrix:

```bash
python3 runner/run_one.py configs/matrices/gazebo_vs_o3de.yaml
```

This will run **2 benchmarks**:
- Same world, Same SLAM, Same Nav2
- Simulator A: Gazebo
- Simulator B: O3DE

**Compare results:**
```bash
cd results/runs/
ls -lt | head -5

# View metrics
cat RUN_GAZEBO/metrics.json
cat RUN_O3DE/metrics.json
```

## 🐛 Troubleshooting

### O3DE Not Found
```
[ERROR] O3DE is not installed!
```
**Solution:** Go to GUI → Tools → Simulators → Install O3DE

### SDF Conversion Failed
```
[ERROR] Failed to convert SDF to O3DE
```
**Solution:** Check that your SDF uses supported geometries (box, cylinder, sphere)

### O3DE Won't Launch
```bash
# Manual test
~/.slam_bench/o3de/build/linux/bin/profile/Editor --project-path=... --level=slam_world
```

### Process Cleanup Issues
```bash
# Nuclear cleanup
pkill -9 -f Editor
pkill -9 -f gzserver
```

## ⚙️ Advanced: GUI Options

In the GUI Details page, you can toggle:
- **Enable Gazebo/O3DE GUI** - Show 3D visualization
- **Enable RViz** - Show ROS visualization
- **Show Results after Run** - Auto-open metrics

These work identically for both Gazebo and O3DE!

## 📈 Expected Performance (Your RX 6950 XT)

**Gazebo:**
- FPS: ~30-60 (limited by ODE physics)
- CPU: 50-80%
- GPU: 10-20% (OpenGL is old)

**O3DE:**
- FPS: ~120-240 (PhysX is faster)
- CPU: 40-60% (PhysX multi-threaded)
- GPU: 30-50% (Vulkan utilizes your card better)

**Metrics should be similar** (same Nav2 logic), but O3DE path might be slightly different due to better physics.

## 🎓 Next Steps

1. Run comparison benchmark
2. Check if O3DE improves your SLAM algorithm's performance
3. Use O3DE for production runs (more stable, better FPS)
4. Customize O3DE rendering settings for photorealistic sensors

Happy benchmarking! 🚀
