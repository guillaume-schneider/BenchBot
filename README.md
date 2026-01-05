# SLAM Bench Orchestrator

![Python](https://img.shields.io/badge/python-3.10-blue)
![ROS 2](https://img.shields.io/badge/ROS%202-Humble-green)
![License](https://img.shields.io/badge/license-MIT-orange)
![Docker](https://img.shields.io/badge/docker-supported-2496ED?logo=docker&logoColor=white)
![Platform](https://img.shields.io/badge/platform-linux-lightgrey)

An automated framework for benchmarking SLAM algorithms (Cartographer, GMapping, SLAM Toolbox, etc.) in dynamic simulation environments (Gazebo, O3DE).

## 🚀 Key Features

*   **Multi-Simulator**: Switch between **Gazebo** (Classic) and **O3DE** (PhysX 5.0).
*   **High-Fidelity Physics**: "Sim-to-Real" tuning with realistic wheel slip, friction, and IMU/Lidar noise ([Details](docs/SIMULATION_REALISM.md)).
*   **Advanced Metrics**:
    *   **Trajectory**: ATE (Absolute Trajectory Error) with automatic alignment
    *   **Map Quality**: Coverage %, IoU, **SSIM (Structural Similarity)**, **Wall Thickness Analysis**
    *   **System**: Real-time CPU Usage %, Max RAM (MB)
    *   **Anomaly Detection**: Stuck robot, TF jumps, massive drift detection
*   **Docker Support**: Full containerization for 100% reproducible benchmarks across environments
*   **Automated Reporting**: One-click PDF generation with trajectory plots, metrics tables, and health indicators
*   **Headless CI**: Run full benchmarks on servers without a display (`runner/run_matrix.py`).
*   **Modern GUI**: Dashboard, Analysis Comparison, 3D Visualizer, Robot Manager, and Settings.
*   **Intelligent Comparison**: Overlay up to 3 trajectories with ground truth and detailed anomaly tooltips.
*   **3D Real-Time Monitoring**: Live LIDAR point cloud, robot pose, and trajectory with "Follow Robot" camera mode.
*   **Stress Testing**: Dynamic sensor degradation (LIDAR noise/range) and actuator limiting.

## 📦 Installation

### Prerequisites
- **OS:** Ubuntu 22.04 LTS
- **ROS 2:** Humble Hawksbill ([Installation Guide](https://docs.ros.org/en/humble/Installation.html))
- **Python:** 3.10+

### Quick Install
```bash
# Clone the repository
git clone https://github.com/YOUR_USERNAME/slam_bench_orchestrator.git
cd slam_bench_orchestrator

# Run automated installer
./install.sh

# Or install manually
pip3 install -r requirements.txt
```

## 🎮 Quick Start

### 1. Launch GUI
```bash
python3 gui/main.py
```
*   **Settings**: Install Simulators (O3DE), toggle Themes (Dark/Light), enable Docker execution, and build container images.
*   **Dashboard**: Select benchmark matrices and monitor live metrics (CPU, RAM, robot pose).
*   **Benchmark**: View results in a sortable table with health indicators.
*   **Comparison**: Overlay up to 3 trajectories with advanced metrics (ATE, Coverage, SSIM, Wall Thickness) and export PDF reports.
*   **3D Visualizer**: Live LIDAR point cloud, robot trajectory, and "Follow Robot" camera mode.
*   **Robot Manager**: Inject noise, limit LIDAR range, or scale motor speeds to test robustness.

### 2. Docker Mode (Reproducible)
Build and run in an isolated container:
```bash
docker-compose build
docker-compose up
```
Or enable Docker execution from the GUI Settings tab.

### 3. Headless Mode (CI/CD)
Run a full benchmark suite without opening any windows:
```bash
python3 runner/run_matrix.py configs/matrices/test_headless_ci.yaml
```

## 📚 Documentation

*   **[Roadmap](docs/ROADMAP.md)**: Future features including auto-tuning and failure injection.
*   **[Setup & Specs](docs/SETUP_AND_SPECS.md)**: Hardware requirements and installation guide.
*   **[Simulators Guide](docs/SIMULATORS.md)**: Installing and using Gazebo vs O3DE.
*   **[Auto-Tuner Guide](docs/AUTO_TUNER_GUIDE.md)**: Optimizing SLAM parameters with AI.
*   **[O3DE Deep Dive](docs/o3de/)**: Comprehensive documentation for O3DE integration.
*   **[Metrics Documentation](docs/METRICS.md)**: Details on ATE, coverage, SSIM, and wall thickness.
*   **[Headless CI](docs/HEADLESS_CI.md)**: Running benchmarks in CLI/CI mode.
*   **[Simulation Realism](docs/SIMULATION_REALISM.md)**: Physics and Sensor noise parameters.
*   **[Troubleshooting Exploration](docs/TROUBLESHOOTING_EXPLORATION.md)**: Solutions for navigation issues.
*   **[Analysis & Comparison](docs/ANALYSIS_GUIDE.md)**: Using the comparison tool, PDF reports, and anomaly detection.
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

## 🤝 Contributing

We welcome contributions! Please see our [Contributing Guide](CONTRIBUTING.md) for details on:
- Code style guidelines
- Development setup
- Pull request process
- Bug reporting

## 📜 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## 🙏 Acknowledgments

- ROS 2 Community for the robotics middleware
- OpenSLAM for GMapping implementation
- Google Cartographer team
- All contributors and testers

## 📧 Contact

For questions or support:
- **Issues:** [GitHub Issues](https://github.com/YOUR_USERNAME/slam_bench_orchestrator/issues)
- **Discussions:** [GitHub Discussions](https://github.com/YOUR_USERNAME/slam_bench_orchestrator/discussions)

---

**Made with ❤️ for the SLAM research community**

