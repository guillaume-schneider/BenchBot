# 📚 SLAM Bench Orchestrator - Complete Documentation Index

Welcome to the comprehensive documentation for the SLAM Bench Orchestrator. This index provides quick access to all documentation resources.

## 🚀 Getting Started

- **[README.md](../README.md)** - Project overview, quick start, and installation
- **[CONTRIBUTING.md](../CONTRIBUTING.md)** - Guide for contributors (code style, PR process)
- **[LICENSE](../LICENSE)** - MIT License terms
- **[ROADMAP.md](ROADMAP.md)** - Future features and development priorities

## 📖 Core Documentation

### Setup & Installation
- **[SETUP_AND_SPECS.md](SETUP_AND_SPECS.md)** - Hardware requirements and detailed installation
- **[AUTOMATED_DEPENDENCIES.md](AUTOMATED_DEPENDENCIES.md)** - Dynamic dependency management

### Simulators
- **[SIMULATORS.md](SIMULATORS.md)** - Gazebo vs O3DE comparison and setup
- **[SIMULATION_REALISM.md](SIMULATION_REALISM.md)** - Physics tuning and sensor noise
- **[o3de/](o3de/)** - Complete O3DE integration documentation

### Metrics & Analysis
- **[METRICS.md](METRICS.md)** - All metrics explained (ATE, SSIM, Wall Thickness, etc.)
- **[ANALYSIS_GUIDE.md](ANALYSIS_GUIDE.md)** - Comparison tool, PDF reports, anomaly detection

### Advanced Features
- **[AUTO_TUNER_GUIDE.md](AUTO_TUNER_GUIDE.md)** - Parameter optimization with AI
- **[ROBOT_HARDWARE_MANAGER.md](ROBOT_HARDWARE_MANAGER.md)** - Sensor degradation and stress testing
- **[MULTI_SLAM_GUIDE.md](MULTI_SLAM_GUIDE.md)** - Running multiple SLAM algorithms

### Execution Modes
- **[HEADLESS_CI.md](HEADLESS_CI.md)** - Running benchmarks in CLI/CI mode
- **Docker** - See [README.md](../README.md#docker-mode-reproducible) for containerization

## 🧪 Testing & Quality

- **[tests/README.md](../tests/README.md)** - Complete testing guide
  - Unit tests (pytest)
  - Integration tests
  - Coverage reports
  - Test markers and fixtures

## 📝 Logging & Debugging

- **[logs/README.md](../logs/README.md)** - Logging system documentation
  - Log file structure
  - Crash reports
  - Viewing and analyzing logs
  - Cleanup and maintenance

- **[examples/logging_examples.py](../examples/logging_examples.py)** - Logging usage examples

## 🔧 Troubleshooting

### Exploration & Navigation
- **[EXPLORATION_FIX_COMPLETE.md](EXPLORATION_FIX_COMPLETE.md)** - ⭐ Guide complet de résolution des problèmes d'exploration
- **[EXPLORATION_QUICK_REFERENCE.md](EXPLORATION_QUICK_REFERENCE.md)** - Référence rapide pour l'exploration
- **[TROUBLESHOOTING_EXPLORATION.md](TROUBLESHOOTING_EXPLORATION.md)** - Navigation et exploration issues
- **[ROBUST_SYNCHRONIZATION.md](ROBUST_SYNCHRONIZATION.md)** - Synchronisation robuste avec probes
- **[GAZEBO_CRASH_ANALYSIS.md](GAZEBO_CRASH_ANALYSIS.md)** - Analyse et résolution des crashes Gazebo
- **[RUN_STOP_STABILIZATION.md](RUN_STOP_STABILIZATION.md)** - ⭐ Stabilisation du cycle RUN/STOP et gestion des processus

### SLAM-Specific
- **[TROUBLESHOOTING_GMAPPING.md](TROUBLESHOOTING_GMAPPING.md)** - GMapping-specific problems
- **[FIXES_EXPLORE_LITE.md](../FIXES_EXPLORE_LITE.md)** - explore_lite configuration fixes
- **[FIXES_TF_PROBLEMS.md](../FIXES_TF_PROBLEMS.md)** - TF transformation issues
- **[ARCHIVE/FIX_PROJECT_RENAME.md](archive/FIX_PROJECT_RENAME.md)** - Fixes for project rename issues

## 📦 Project Structure

```
slam_bench_orchestrator/
├── gui/                    # PyQt5 GUI application
│   ├── main.py            # Entry point
│   ├── pages/             # Dashboard, Comparison, Visualizer, etc.
│   └── worker.py          # Background task runner
├── runner/                # Core orchestration logic
│   ├── orchestrator.py    # Main benchmark runner
│   ├── run_one.py         # Single run executor
│   └── run_matrix.py      # Batch runner
├── evaluation/            # Metrics calculation
│   └── metrics.py         # ATE, SSIM, Coverage, Wall Thickness
├── tools/                 # Utility scripts
│   ├── benchmark.py       # ATE calculation
│   ├── report_generator.py # PDF reports
│   └── simulator_manager.py # O3DE/Gazebo management
├── utils/                 # Shared utilities
│   └── logger.py          # Centralized logging system
├── configs/               # YAML configurations
│   ├── matrices/          # Benchmark suites
│   ├── slams/             # SLAM profiles
│   ├── datasets/          # Simulation scenarios
│   └── params/            # Algorithm parameters
├── tests/                 # Unit and integration tests
├── logs/                  # Log files and crash reports
├── results/               # Benchmark outputs
├── docs/                  # Documentation (you are here!)
└── examples/              # Usage examples
```

## 🎯 Quick Reference by Task

### I want to...

#### **Run a Benchmark**
1. [Quick Start Guide](../README.md#quick-start)
2. [Headless Mode](HEADLESS_CI.md) (for servers)
3. [Docker Mode](../README.md#docker-mode-reproducible) (for reproducibility)

#### **Analyze Results**
1. [Metrics Documentation](METRICS.md)
2. [Analysis Guide](ANALYSIS_GUIDE.md)
3. [PDF Reports](ANALYSIS_GUIDE.md#pdf-reports)

#### **Add a New SLAM Algorithm**
1. [Multi-SLAM Guide](MULTI_SLAM_GUIDE.md)
2. [SLAM Configuration](MULTI_SLAM_GUIDE.md#adding-a-new-slam)

#### **Tune SLAM Parameters**
1. [Auto-Tuner Guide](AUTO_TUNER_GUIDE.md)
2. [Parameter Files](../configs/params/)

#### **Test Robustness**
1. [Robot Hardware Manager](ROBOT_HARDWARE_MANAGER.md)
2. [Sensor Degradation](ROBOT_HARDWARE_MANAGER.md#sensor-degradation)

#### **Contribute Code**
1. [Contributing Guide](../CONTRIBUTING.md)
2. [Testing Guide](../tests/README.md)
3. [Logging Examples](../examples/logging_examples.py)

#### **Debug Issues**
1. [Troubleshooting Exploration](TROUBLESHOOTING_EXPLORATION.md)
2. [Troubleshooting GMapping](TROUBLESHOOTING_GMAPPING.md)
3. [Log Files](../logs/README.md)
4. [Crash Reports](../logs/README.md#crash-reports)

## 📊 Metrics Reference

| Metric | Description | Goal | Documentation |
|--------|-------------|------|---------------|
| **ATE RMSE** | Absolute Trajectory Error | Lower is better | [METRICS.md](METRICS.md#1-ate-absolute-trajectory-error) |
| **Coverage** | % of GT area explored | Higher is better | [METRICS.md](METRICS.md#2-map-coverage) |
| **SSIM** | Structural similarity | Higher is better (>0.8) | [METRICS.md](METRICS.md#4-ssim-structural-similarity-index) |
| **Wall Thickness** | Wall sharpness | Lower is better (<10cm) | [METRICS.md](METRICS.md#5-wall-thickness-analysis) |
| **IoU** | Occupancy overlap | Higher is better | [METRICS.md](METRICS.md#3-iou-intersection-over-union) |
| **CPU Usage** | Peak CPU % | Lower is better | [METRICS.md](METRICS.md#7-max-cpu-usage) |
| **RAM Usage** | Peak RAM (MB) | Lower is better | [METRICS.md](METRICS.md#8-max-ram-usage) |

## 🔗 External Resources

- **ROS 2 Humble Documentation**: https://docs.ros.org/en/humble/
- **Gazebo Classic**: http://gazebosim.org/
- **O3DE**: https://www.o3de.org/
- **Cartographer**: https://google-cartographer.readthedocs.io/
- **SLAM Toolbox**: https://github.com/SteveMacenski/slam_toolbox

## 📧 Support

- **Issues**: [GitHub Issues](https://github.com/YOUR_USERNAME/slam_bench_orchestrator/issues)
- **Discussions**: [GitHub Discussions](https://github.com/YOUR_USERNAME/slam_bench_orchestrator/discussions)
- **Email**: For private inquiries

## 🏆 Recent Updates

### v1.0 (2026-01-05)
- ✅ Advanced metrics (SSIM, Wall Thickness)
- ✅ Docker support with GUI integration
- ✅ PDF report generation
- ✅ Centralized logging system
- ✅ Comprehensive test suite (70+ tests)
- ✅ Anomaly detection
- ✅ 3D visualizer with "Follow Robot" mode

See [ROADMAP.md](ROADMAP.md) for upcoming features.
