# SLAM Bench Orchestrator

An automated framework for benchmarking SLAM algorithms (Cartographer, GMapping, SLAM Toolbox, etc.) in dynamic simulation environments (Gazebo, O3DE).

## 🚀 Key Features

*   **Multi-Simulator**: Switch between **Gazebo** (Classic) and **O3DE** (PhysX 5.0).
*   **High-Fidelity Physics**: "Sim-to-Real" tuning with realistic wheel slip, friction, and IMU/Lidar noise ([Details](docs/SIMULATION_REALISM.md)).
*   **Automated Metrics**:
    *   **Trajectory**: ATE (Absolute Trajectory Error)
    *   **Map Quality**: Coverage %, IoU (Intersection over Union), Path Length
    *   **System**: CPU Usage %, Max RAM (MB)
*   **Headless CI**: Run full benchmarks on servers without a display (`runner/run_matrix.py`).
*   **Modern GUI**: Dashboard, Analysis Comparison, and Robot Hardware Manager.
*   **Intelligent Analysis**: Comparison tool with trajectory overlay and heuristic anomaly detection.
*   **Stress Testing**: Real-time sensor degradation (LIDAR noise/range) and actuator limiting.

## 📦 Quick Start

### 1. Launch GUI
```bash
python3 gui/main.py
```
*   **Settings**: Install Simulators (O3DE) and toggle Themes (Dark/Light).
*   **Dashboard**: Select `modeA_validate_loop` or other matrices.
*   **Benchmark**: View results in a sortable table.
*   **Comparison**: Overlay trajectories from 3 runs on a ground truth map and detect anomalies.
*   **Robot Manager**: Inject noise, limit LIDAR range, or scale motor speeds to test robustness.

### 2. Headless Mode (CI/CD)
Run a full benchmark suite without opening any windows:
```bash
python3 runner/run_matrix.py configs/matrices/test_headless_ci.yaml
```

## 📚 Documentation

*   **[Setup & Specs](docs/SETUP_AND_SPECS.md)**: Hardware requirements and installation guide.
*   **[Simulators Guide](docs/SIMULATORS.md)**: Installing and using Gazebo vs O3DE.
*   **[Auto-Tuner Guide](docs/AUTO_TUNER_GUIDE.md)**: Optimizing SLAM parameters with AI.
*   **[O3DE Deep Dive](docs/o3de/)**: Comprehensive documentation for O3DE integration.
*   **[Metrics Documentation](docs/METRICS.md)**: Details on ATE, coverage, and system metrics.
*   **[Headless CI](docs/HEADLESS_CI.md)**: Running benchmarks in CLI/CI mode.
*   **[Simulation Realism](docs/SIMULATION_REALISM.md)**: Physics and Sensor noise parameters.
*   **[Troubleshooting Exploration](docs/TROUBLESHOOTING_EXPLORATION.md)**: Solutions for navigation issues.
*   **[Analysis & Comparison](docs/ANALYSIS_GUIDE.md)**: Using the comparison tool and understanding anomaly detection.
*   **[Robot Hardware Manager](docs/ROBOT_HARDWARE_MANAGER.md)**: Configuring sensor noise and motor limits for stress tests.
*   **[Automated Dependencies](docs/AUTOMATED_DEPENDENCIES.md)**: Dynamic Git cloning and building for SLAMs.
*   **[Archive & Logs](docs/archive/)**: Historical session logs and debugging notes.



## 🛠️ Project Structure

*   `gui/`: PyQT5 interface (Main, Dashboard, Settings, Benchmark).
*   `runner/`: Core orchestration logic (`orchestrator.py`, `run_matrix.py`).
*   `configs/`:
    *   `matrices/`: Benchmark suites.
    *   `slams/`: SLAM algorithm profiles.
    *   `datasets/`: Simulation scenarios.
*   `results/`: Runs, logs, bags, and generated metrics.

