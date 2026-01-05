# System Requirements & Tech Stack

This document outlines the software architecture, hardware recommendations, and setup procedures for the SLAM Bench Orchestrator.

## 🛠️ Technology Stack

### Core Framework
- **OS**: Ubuntu 22.04 LTS (Jammy Jellyfish) - *Required for ROS 2 Humble*.
- **Middleware**: [ROS 2 Humble Hawksbill](https://docs.ros.org/en/humble/index.html).
- **Languages**: 
    - **Python 3.10+**: Orchestrator logic, GUI, metrics calculation.
    - **C++**: High-performance SLAM backends (GMapping, SLAM Toolbox).

### Simulation Engines
- **Gazebo Classic**: Standard ROS simulation for lightweight benchmarking.
- **O3DE (Open 3D Engine)**: High-fidelity simulation with PhysX 5.0 and Vulkan rendering.

### Key Libraries (Python)
- **PyQt5**: Modern desktop interface.
- **Matplotlib/NumPy**: Data visualization and trajectory analysis.
- **ReportLab**: Automated PDF report generation.
- **psutil**: Real-time system resource monitoring (CPU/RAM).
- **rosbag2_py**: Direct reading of ROS 2 sqlite3 databases.

---

## 💻 Hardware Specifications

Benchmarking SLAM requires significant resources, especially when running the simulator and SLAM stack simultaneously.

| Component | Minimum (Gazebo) | Recommended (O3DE + SLAM) |
| :--- | :--- | :--- |
| **CPU** | Quad-core (Intel i5 / AMD R5) | 8-core+ (Intel i7 / AMD R7) |
| **RAM** | 8 GB | 16 GB - 32 GB |
| **GPU** | Integrated Graphics | **NVIDIA RTX 2060+ / AMD RX 5700+** |
| **Disk** | 5 GB Free | 50 GB Free (O3DE is bulky) |
| **Graphics API** | OpenGL | **Vulkan Support (Required for O3DE)** |

> **Note**: For O3DE, a dedicated GPU with proprietary drivers (Nvidia/AMD) is highly recommended for stable physics and frame rates.

---

## 🏗️ System Setup

### 1. ROS 2 Environment
Ensure ROS 2 Humble is installed and sourced:
```bash
source /opt/ros/humble/setup.bash
```

### 2. SLAM & Navigation Dependencies
Install the required ROS 2 binary packages:
```bash
sudo apt update
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup \
                 ros-humble-slam-toolbox ros-humble-cartographer-ros \
                 ros-humble-turtlebot3-msgs ros-humble-turtlebot3-simulations
```

### 3. Python Environment
Install the required Python modules:
```bash
pip install PyQt5 pyyaml numpy matplotlib reportlab psutil
```

### 4. GMapping Path (Legacy Support)
If you intend to use the patched GMapping included in this project:
```bash
cd deps/gmapping_ws
colcon build --symlink-install
source install/setup.bash
```

---

## 📁 Workspace Architecture

```text
slam_bench_orchestrator/
├── configs/          # Configuration files (YAML/LUA)
│   ├── matrices/     # Benchmark suites definition
│   ├── slams/        # SLAM algorithm profiles
│   └── datasets/     # Simulation scenarios
├── runner/           # Core Orchestration (Headless support)
├── gui/              # PyQt5 Dashboard and Benchmark views
├── tools/            # Analysis & Reporting scripts
├── results/          # Artifacts (Bags, Metrics, Plots)
└── docs/             # Technical documentation & Guides
```

---

## 🔌 Network & Security
- **ROS_DOMAIN_ID**: Ensure all benchmark runs use a unique domain ID if multiple users are on the same network.
- **Firewall**: UFW should allow local traffic for ROS 2 discovery (DDS).
