# SLAM Bench Orchestrator

An automated framework for benchmarking SLAM algorithms (Cartographer, GMapping, SLAM Toolbox, etc.) in dynamic simulation environments (Gazebo, O3DE).

## Quick Start

1. **Launch the GUI**:
   ```bash
   python3 gui/main.py
   ```
2. **Select a Benchmark Matrix**:
   - Go to the Dashboard.
   - Choose a configuration (e.g., `modeA_validate_loop`).
3. **Run**:
   - Click "Run Benchmark".
   - Monitor progress and logs in real-time.

## Features

- **Multi-Simulator Support**: Seamlessly switch between Gazebo and O3DE.
- **Automated Metrics**: Computes ATE (Trajectory Error) and Map Quality (Coverage, IoU) automatically.
- **Resilience**: Automatic cleanup of zombie processes and handling of failed runs.
- **Visualization**: Built-in tools for visualizing Ground Truth vs Generated Maps.

## Documentation

- **[Troubleshooting Exploration](docs/TROUBLESHOOTING_EXPLORATION.md)**: Solutions for stuck robots and stability issues.
- **[Multi-SLAM Guide](docs/MULTI_SLAM_GUIDE.md)**: How to add new SLAM algorithms.
- **[O3DE Guide](docs/O3DE_README.md)**: Setup and usage with O3DE.
