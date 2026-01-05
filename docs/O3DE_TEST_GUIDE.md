# 🚀 O3DE Quick Test Guide

## ⚡ Quick Test (5 minutes)

```bash
cd /home/schneigu/Projects/slam_bench_orchestrator
python3 tests/test_o3de_headless.py
```

**Expected output**:
- ✅ O3DE installed
- ✅ Project found
- ✅ GameLauncher found
- 🚀 Starting O3DE...
- ✅ Process started
- ✅ Process stable after 10s
- ✅ Test successful!

---

## 🎯 Full Benchmark Test (10-15 minutes)

```bash
python3 runner/run_one.py configs/matrices/o3de_test.yaml
```

**What happens**:
1. Asset Processor starts (30s wait)
2. O3DE launches in headless mode
3. Nav2 and exploration start
4. Robot explores the world
5. Data collected and metrics computed
6. Clean shutdown

**Check results**:
```bash
ls -lt results/runs/ | head -3
cat results/runs/LATEST/metrics.json
```

---

## 🆚 Gazebo vs O3DE Comparison (30 minutes)

```bash
python3 runner/run_one.py configs/matrices/gazebo_vs_o3de.yaml
```

This runs TWO benchmarks:
- Same world, same SLAM, same duration
- Simulator A: Gazebo
- Simulator B: O3DE

**Compare performance**:
```bash
cd results/runs
# Find the two latest runs
ls -lt | head -5

# Compare metrics
cat RUN_GAZEBO/metrics.json
cat RUN_O3DE/metrics.json
```

---

## 🐛 Troubleshooting

### Test fails immediately
**Check**: Is O3DE installed?
```bash
ls ~/.slam_bench/o3de/build/linux/bin/profile/AssetProcessor
```

### Asset Processor errors
**Solution**: Run it manually first:
```bash
cd ~/.slam_bench/o3de/projects/model_o3de_project
~/.slam_bench/o3de/build/linux/bin/profile/AssetProcessor
# Wait for "Idle" status, then Ctrl+C
```

### ROS2 topics not publishing
**Check**: Are processes running?
```bash
ros2 topic list
# Should see: /scan, /odom, /tf, /map, /cmd_vel
```

### Process crashes
**Check logs**:
```bash
cat results/runs/LATEST/logs/o3de_sim.log
```

---

## 📚 Documentation

- **Usage Guide**: `docs/O3DE_HEADLESS_MODE.md`
- **Status**: `docs/O3DE_STATUS_AND_ROADMAP.md`
- **Implementation**: `docs/O3DE_HEADLESS_IMPLEMENTATION_SUMMARY.md`
- **Build Results**: `docs/O3DE_BUILD_TEST_RESULTS.md`

---

## ✅ Success Checklist

- [ ] Quick test passes
- [ ] Full benchmark completes
- [ ] ROS2 topics publish
- [ ] Metrics computed
- [ ] Clean shutdown
- [ ] No zombie processes

---

## 🎓 What You're Testing

1. **Asset Processor** - Auto-starts in background
2. **Headless Mode** - Runs without display
3. **GameLauncher** - Uses correct executable
4. **Process Management** - Clean startup/shutdown
5. **ROS2 Integration** - Topics publish correctly
6. **Orchestrator Integration** - Works through CLI/GUI

**Ready to go!** 🚀
