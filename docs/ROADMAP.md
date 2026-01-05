# 🗺️ SLAM Bench Orchestrator - Roadmap

This document outlines the future development directions for the SLAM Bench Orchestrator, focusing on advanced automation, robustness testing, and deeper analytical capabilities.

---

## 🎯 1. Optimisation Automatique des Paramètres (Hyper-Tuning)

**Status:** 🔮 Planned

**Objective:**  
Automatically discover optimal SLAM parameters through systematic exploration, eliminating manual trial-and-error.

**Current State:**  
Users must manually define fixed parameter sets in configuration files and compare results post-hoc.

**Proposed Solution:**

### Features
- **Auto-Tuner Tab:** New GUI section dedicated to parameter optimization
- **Search Strategies:**
  - **Grid Search:** Exhaustive exploration of discrete parameter combinations
  - **Bayesian Optimization:** Intelligent sampling using Gaussian Processes to minimize ATE/maximize coverage
  - **Random Search:** Baseline comparison method
- **Multi-Objective Optimization:** Balance trade-offs (e.g., accuracy vs. computational cost)

### Implementation Plan
1. **Parameter Space Definition:**
   - YAML schema for defining ranges (e.g., `linearUpdate: [0.1, 0.5, 1.0]`)
   - Support for continuous and discrete parameters
   
2. **Optimization Engine:**
   - Integration with `scikit-optimize` or `Optuna`
   - Parallel run execution for faster convergence
   
3. **Visualization:**
   - Real-time convergence plots
   - Parameter sensitivity heatmaps
   - Pareto frontier for multi-objective cases

4. **Result Export:**
   - Best configuration auto-saved to `configs/optimized/`
   - Detailed optimization report (PDF/HTML)

### Use Cases
- Tuning GMapping's `linearUpdate` and `angularUpdate` for specific environments
- Finding optimal Cartographer resolution parameters
- Balancing exploration speed vs. map quality in `explore_lite`

---

## 🗺️ 2. Métriques de Qualité de Carte Avancées

**Status:** ✅ Completed (v1.0)

**Implemented:**
- **SSIM (Structural Similarity Index):** Measures geometric fidelity beyond pixel-level IoU
- **Wall Thickness Analysis:** Detects map "blur" caused by drift or sensor noise
- **Automatic Integration:** Metrics computed post-run and displayed in comparison tables

**Future Enhancements:**
- **Topological Correctness:** Verify room connectivity and corridor structure
- **Loop Closure Quality:** Analyze drift before/after loop closures

---

## 🧨 3. Injection d'Échecs Dynamiques (Failure Injection)

**Status:** 🔮 Planned

**Objective:**  
Test SLAM robustness against real-world anomalies and edge cases.

**Current State:**  
The Robot Hardware Manager provides static sensor degradation (constant noise, range limits). Real-world scenarios involve sudden, unpredictable failures.

**Proposed Solution:**

### Failure Modes

#### 3.1 Kidnapped Robot Problem
- **Description:** Teleport robot to random location mid-run
- **Test:** SLAM recovery and relocalization capabilities
- **Implementation:**
  - Gazebo service call to reset robot pose
  - Configurable trigger (time-based, coverage-based, random)
  - Metrics: Time to recovery, final ATE impact

#### 3.2 Sensor Blackout
- **Description:** Temporarily disable LIDAR/camera during critical moments
- **Test:** Dead reckoning accuracy, map consistency
- **Implementation:**
  - Modify `sensor_degrader.py` to support temporal patterns
  - Blackout scenarios: corners, narrow corridors, high-speed sections
  - Metrics: Drift during blackout, post-recovery convergence

#### 3.3 Dynamic Obstacles
- **Description:** Spawn moving "ghost" objects or pedestrians
- **Test:** Robustness to non-static environments
- **Implementation:**
  - Gazebo model spawning via ROS services
  - Patterns: crossing paths, following robot, random walks
  - Metrics: False positive walls, map stability

#### 3.4 Odometry Corruption
- **Description:** Inject wheel slip, IMU drift, or encoder glitches
- **Test:** Sensor fusion resilience
- **Implementation:**
  - Probabilistic noise injection in `/odom` topic
  - Scenarios: slippery surfaces, sudden impacts
  - Metrics: ATE degradation, loop closure dependency

### Configuration Example
```yaml
failure_injection:
  enabled: true
  scenarios:
    - type: kidnapped_robot
      trigger: coverage_threshold
      threshold: 0.5  # Trigger at 50% coverage
      teleport_distance: [5.0, 10.0]  # meters
      
    - type: sensor_blackout
      trigger: time_interval
      start: 30.0  # seconds
      duration: 3.0
      affected_sensors: [lidar]
      
    - type: dynamic_obstacle
      count: 3
      spawn_rate: 0.1  # Hz
      behavior: random_walk
```

### GUI Integration
- **Failure Injection Tab:** Configure scenarios visually
- **Live Indicators:** Show active failures during run
- **Comparison Mode:** Compare same SLAM with/without failures

---

## 🐳 4. Docker Container Support

**Status:** ✅ Completed (v1.0)

**Implemented:**
- Full containerization with `Dockerfile` and `docker-compose.yml`
- GUI toggle for Docker execution in Settings
- One-click image building from interface
- Automatic volume mounting for results persistence

---

## 📊 5. Advanced Reporting & Analytics

**Status:** ✅ Completed (v1.0)

**Implemented:**
- Automated PDF report generation with trajectory plots
- Anomaly detection (stuck robot, TF jumps, massive drift)
- Comparison tables with health indicators

**Future Enhancements:**
- **Interactive HTML Reports:** Zoomable plots, filterable tables
- **Time-Series Analysis:** CPU/RAM trends, coverage progression
- **Statistical Summaries:** Mean/std across multiple seeds

---

## 🔄 6. Continuous Integration & Regression Testing

**Status:** 🟡 Partial

**Current:**
- Headless execution support (`run_matrix.py`)
- Basic CI-friendly configuration (`test_headless_ci.yaml`)

**Planned:**
- GitHub Actions workflow for automated testing
- Performance regression detection
- Benchmark result archiving and trending

---

## 🌐 7. Multi-Robot SLAM

**Status:** 🔮 Future

**Objective:**  
Benchmark collaborative SLAM with multiple robots.

**Challenges:**
- Map merging evaluation
- Communication bandwidth simulation
- Coordination strategy comparison

---

## 🎮 8. Extended Simulator Support

**Status:** 🟡 Partial

**Current:**
- Gazebo Classic (full support)
- O3DE (experimental, sensor integration pending)

**Planned:**
- **Isaac Sim:** NVIDIA's photorealistic simulator
- **Webots:** Lightweight alternative
- **Carla:** Urban driving scenarios

---

## 📝 Contributing to the Roadmap

Have ideas or want to implement a feature? Check the [CONTRIBUTING.md](../CONTRIBUTING.md) guide or open an issue on GitHub.

---

**Last Updated:** 2026-01-05  
**Maintainer:** SLAM Bench Team
