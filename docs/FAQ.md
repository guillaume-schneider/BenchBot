# Frequently Asked Questions (FAQ)

This page answers the most common questions about BenchBot.

---

## Installation and Configuration

### Which simulator should I choose to start?

**Gazebo Classic** is recommended for beginners:

- ✅ Stable and well-documented
- ✅ Less resource-intensive
- ✅ Large ROS 2 community
- ✅ Compatible with most SLAM packages

**O3DE** is recommended for:

- Realistic graphics
- Advanced physics
- Visual tests and demonstrations

### How long does a typical benchmark take?

**Complete Timeline**:

- **Preparation**: 5-10 minutes (first time)
- **Execution**: 2-5 minutes per run
- **Evaluation**: 30 seconds - 2 minutes
- **Analysis**: 5-10 minutes (multi-run comparison)

**Total for a simple run**: ~5-7 minutes

### What are the minimal dependencies?

**Mandatory**:

- Python 3.8+
- ROS 2 Humble
- A simulator (Gazebo or O3DE)
- At least one SLAM algorithm (cartographer, slam_toolbox, etc.)

**Optional**:

- Nav2 (for autonomous navigation)
- MkDocs (for local documentation)

### How to configure my first benchmark?

**Step 1**: Create `matrix.yaml`
```yaml
matrix:
  include:
    - dataset: warehouse
      slam: cartographer
      simulator: gazebo
```

**Step 2**: Launch
```bash
python -m gui.main  # Graphical Interface
# OR
python -m runner.orchestrator --config matrix.yaml  # CLI
```

---

## Metrics and Results

### What is the difference between IoU and SSIM?

| Metric | Measure | Utility |
|--------|---------|---------|
| **IoU** | Global similarity (overlap) | Coverage precision |
| **SSIM** | Structural coherence (shapes) | Detail quality |

**Example**:

- IoU = 0.85: 85% of the GT map is correctly covered
- SSIM = 0.90: Structures (walls, corridors) are well preserved

### What is a good score?

| Metric | Excellent | Good | Acceptable | Bad |
|--------|-----------|------|------------|-----|
| **IoU** | > 0.85 | 0.70-0.85 | 0.60-0.70 | < 0.60 |
| **SSIM** | > 0.90 | 0.80-0.90 | 0.70-0.80 | < 0.70 |
| **ATE** | < 0.10m | 0.10-0.20m | 0.20-0.30m | > 0.30m |
| **Coverage** | > 95% | 85-95% | 75-85% | < 75% |

### Why are all my metrics at 0?

**Possible Causes**:

1. **Missing GT Map**
   - **Solution**: Rerun, it will be generated automatically

2. **Empty SLAM Map**
   - **Verify**: The `/map` topic is publishing data
   - **Solution**: Check SLAM node logs

3. **Bad Alignment**
   - **Cause**: Different map origins
   - **Solution**: Alignment is automatic, check evaluation logs

### How to interpret ATE (Absolute Trajectory Error)?

**Definition**: Average localization error of the robot compared to ground truth.

**Interpretation**:

- **0.05m**: Excellent precision (5cm)
- **0.15m**: Good precision (15cm)
- **0.30m**: Acceptable precision (30cm)
- **> 0.50m**: Localization problem

**Note**: ATE strongly depends on the environment and the sensor used.

---

## Troubleshooting

### My benchmark fails in WAIT_READY, what to do?

**Symptoms**: Timeout after 60s, state stuck at WAIT_READY

**Possible Causes**:

1. **Topic `/map` not published**
   ```bash
   ros2 topic list | grep map
   ros2 topic hz /map
   ```

2. **Insufficient `/odom` frequency**
   ```bash
   ros2 topic hz /odom  # Must be > 5Hz
   ```

3. **Missing TF `map → base_link`**
   ```bash
   ros2 run tf2_tools view_frames
   ```

**Solutions**:

- Check logs: `logs/RUN_XXX/orchestrator.log`
- Increase probe timeout in configuration
- Verify SLAM is launched

### The simulator does not launch

**Gazebo**:
```bash
# Check installation
gazebo --version

# Test manually
gazebo worlds/warehouse.world
```

**O3DE**:
```bash
# Check installation path
ls ~/O3DE/bin/o3de

# Check environment variables
echo $O3DE_PROJECT_PATH
```

### Zombie processes persist after a crash

**Symptom**: Ports occupied, active `gzserver` processes

**Solution**:
```bash
# Clean all Gazebo processes
pkill -9 gzserver
pkill -9 gzclient

# Clean all ROS 2 processes
pkill -9 ros2
```

**Prevention**: The orchestrator uses process groups (`os.setsid`) to avoid this issue.

### Evaluation fails with "No map data"

**Causes**:

1. **Empty or corrupt rosbag**
   ```bash
   ros2 bag info results/runs/RUN_XXX/rosbag2/
   ```

2. **Topic `/map` not recorded**
   - Check `rosbag_topics` configuration in YAML

3. **Run duration too short**
   - SLAM didn't have time to publish a map
   - **Solution**: Increase `run_duration` to 60s minimum

---

## Advanced Features

### How to use the Autotuner?

**Minimal Configuration**:
```yaml
autotuner:
  enabled: true
  algorithm: bayesian_optimization
  target_metric: iou
  max_iterations: 20
  
  parameters:
    - name: slam.resolution
      type: float
      range: [0.025, 0.1]
```

**Launch**:
```bash
python -m runner.orchestrator --config matrix.yaml --autotuner
```

**Result**: File `config_optimized.yaml` with the best parameters

### How to simulate a noisy sensor?

**Configuration**:
```yaml
degradation:
  enabled: true
  range_sensor:
    noise_std: 0.05  # 5cm Gaussian noise
    noise_type: gaussian
```

**Use Case**: Test SLAM robustness against a low-cost sensor

### How to compare multiple SLAMs?

**Test Matrix**:
```yaml
matrix:
  include:
    - slam: [cartographer, slam_toolbox, rtabmap]
      dataset: warehouse
```

**Result**: 3 automatic runs + comparative PDF report

### How to test different degradation levels?

**Example: Noise Calibration**:
```yaml
matrix:
  include:
    - slam: cartographer
      degradation:
        range_sensor:
          noise_std: [0.01, 0.02, 0.05, 0.1]
```

**Result**: 4 runs with performance vs noise graph

---

## Workflow and Best Practices

### What is the difference between GUI and CLI?

| Mode | Pros | Cons |
|------|------|------|
| **GUI** | Visual interface, real-time monitoring | Requires graphical display |
| **CLI** | Automation, CI/CD, headless | No real-time visualization |

**Recommendation**:

- **GUI**: Development, tests, demonstrations
- **CLI**: Production, CI/CD, massive benchmarks

### How to organize my results?

**Recommended Structure**:
```
results/
├── runs/
│   ├── RUN_20260108_150000/  # One folder per run
│   │   ├── config_resolved.yaml
│   │   ├── rosbag2/
│   │   ├── metrics.json
│   │   └── logs/
│   └── ...
└── reports/
    ├── comparison_slam.pdf
    └── optimization_history.pdf
```

### How many runs for a reliable benchmark?

**Recommendations**:

- **Quick Test**: 1 run (functional validation)
- **Comparison**: 3 runs per configuration (mean + standard deviation)
- **Publication**: 5-10 runs (robust statistics)

**Note**: Reproducibility is guaranteed by `config_resolved.yaml`

### How to share my results?

**Files to share**:

1. `config_resolved.yaml`: Exact configuration
2. `metrics.json`: Numerical results
3. `report.pdf`: Visual report
4. (Optional) `rosbag2/`: Raw data (large)

**Recommended Format**: `.tar.gz` archive of the `RUN_XXX` folder

---

## Next Steps

- **[System Overview](system_overview.md)**: Architecture overview
- **[Orchestrator Architecture](orchestrator_architecture.md)**: State machine and probes
- **[Tools](tools.md)**: Infrastructure and advanced features
- **[Evaluation Logic](evaluation_logic.md)**: Detailed metrics
