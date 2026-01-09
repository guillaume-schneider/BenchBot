<p align="center">
  <img src="docs/assets/banner.png" alt="BenchBot Banner" width="100%">
</p>

# BenchBot

<p align="center">
  <a href="https://benchbot.guillaumeschneider.fr">
    <img src="https://img.shields.io/badge/Docs-Online-blue?style=for-the-badge&logo=materialformkdocs" alt="Documentation">
  </a>
  <img src="https://img.shields.io/badge/ROS%202-Humble-green?style=for-the-badge&logo=ros" alt="ROS 2 Humble">
  <img src="https://img.shields.io/badge/Python-3.10+-blue?style=for-the-badge&logo=python" alt="Python 3.10+">
  <img src="https://img.shields.io/badge/License-MIT-orange?style=for-the-badge" alt="License">
</p>

<p align="center">
  <strong>The Comprehensive Ecosystem for ROS 2 Navigation & SLAM Benchmarking</strong>
</p>

---

## 💡 What is BenchBot?

**BenchBot** is a complete lifecycle ecosystem for professional ROS 2 development. From initial integration to final validation, it empowers teams to master their navigation stack.

*   **🧩 Integrate**: Plug in any SLAM algorithm (Cartographer, SLAM Toolbox, GMapping) with a modular plugin system.
*   **⚙️ Optimize**: Use the **AI Auto-Tuner** to automatically discover the perfect parameters for your robot.
*   **📈 Monitor**: Track evolution with industrial-grade metrics (**ATE**, **SSIM**, **Coverage**) over time.
*   **✅ Validate**: Ensure production readiness with automated CI/CD pipelines and reproducible Docker environments.

---

## 🚀 Key Features

*   **Multi-Simulator**: Switch between **Gazebo** (Classic) and **O3DE** (PhysX 5.0).
*   **High-Fidelity Physics**: "Sim-to-Real" tuning with realistic wheel slip, friction, and IMU/Lidar noise ([Details](https://benchbot.guillaumeschneider.fr/SIMULATION_REALISM/)).
*   **Advanced Metrics**:
    *   **Trajectory**: ATE (Absolute Trajectory Error) with automatic alignment
    *   **Map Quality**: Coverage %, IoU, **SSIM (Structural Similarity)**, Wall Thickness Analysis
    *   **System**: Real-time CPU Usage %, Max RAM (MB)
    *   **Anomaly Detection**: Stuck robot, TF jumps, massive drift detection
*   **Docker Support**: Full containerization for 100% reproducible benchmarks across environments
*   **Automated Reporting**: One-click PDF generation with trajectory plots, metrics tables, and health indicators
*   **Centralized Logging**: Rotating file logs, colored console output, automatic crash reports (JSON)
*   **Comprehensive Testing**: 70+ unit tests with pytest, 80%+ code coverage, CI/CD ready
*   **Headless CI**: Run full benchmarks on servers without a display (`runner/run_matrix.py`).
*   **Modern GUI**: Dashboard, Analysis Comparison, 3D Visualizer, Robot Manager, and Settings.
*   **Intelligent Comparison**: Overlay up to 3 trajectories with ground truth and detailed anomaly tooltips.
*   **3D Real-Time Monitoring**: Live LIDAR point cloud, robot pose, and trajectory with "Follow Robot" camera mode.
*   **Stress Testing**: Dynamic sensor degradation (LIDAR noise/range) and actuator limiting.

---

## 📖 Documentation

Full documentation is available at **[https://benchbot.guillaumeschneider.fr](https://benchbot.guillaumeschneider.fr)**.

### Quick Links
*   [🚀 Quick Start](https://benchbot.guillaumeschneider.fr/QUICK_START/)
*   [📦 Installation Guide](https://benchbot.guillaumeschneider.fr/INSTALLATION/)
*   [📊 Metrics Explained](https://benchbot.guillaumeschneider.fr/METRICS/)
*   [🎮 Simulators (Gazebo vs O3DE)](https://benchbot.guillaumeschneider.fr/SIMULATORS/)
*   [🧠 AI Auto-Tuner](https://benchbot.guillaumeschneider.fr/AUTO_TUNER_GUIDE/)
*   [🖥️ Headless Mode (CI/CD)](https://benchbot.guillaumeschneider.fr/HEADLESS_CI/)

---

## 📦 Installation

### Prerequisites
*   **OS**: Ubuntu 22.04 LTS
*   **ROS 2**: Humble Hawksbill
*   **Python**: 3.10+

### Quick Install
```bash
# Clone the repository
git clone https://github.com/guillaume-schneider/BenchBot.git
cd BenchBot

# Run automated installer
./install.sh
```

For detailed instructions, see the [Installation Guide](https://benchbot.guillaumeschneider.fr/INSTALLATION/).

---

## 🤝 Contributing

We welcome contributions! Please see our [Contributing Guide](https://benchbot.guillaumeschneider.fr/CONTRIBUTING/) for details.

## 📜 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.
