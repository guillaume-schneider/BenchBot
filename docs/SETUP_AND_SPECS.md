---
icon: material/laptop
---

# 🖥️ Setup & Specifications

Hardware and software requirements to ensure reliable benchmarking.

## 💻 Hardware Requirements

Select the profile that matches your simulation needs.

| Component | Minimum (2D Lidar) | Recommended (3D/O3DE) |
| :--- | :--- | :--- |
| **CPU** | Quad-core (i5 or Ryzen 5) | 8-core+ (i7/i9 or Ryzen 7/9) |
| **RAM** | 8 GB | 32 GB+ |
| **GPU** | Integrated Graphics | NVIDIA RTX 3060+ / AMD RX 6600+ |
| **Storage** | 20 GB free space | 100 GB NVMe SSD |

!!! warning "GPU Requirement for O3DE"
    O3DE (Open 3D Engine) heavily relies on GPU raytracing (Vulkan/DX12). Use a dedicated **NVIDIA** or **AMD** card with updated drivers for best performance.

---

## 🐧 Software Environment

### Operating System

| OS | Version | Support Status |
| :--- | :--- | :--- |
| ![Ubuntu](https://img.shields.io/badge/Ubuntu-E95420?style=for-the-badge&logo=ubuntu&logoColor=white) | **22.04 LTS** (Jammy) | ✅ **Official Support** |
| ![Windows](https://img.shields.io/badge/Windows-0078D6?style=for-the-badge&logo=windows&logoColor=white) | 10 / 11 | ❌ Not Supported (Use WSL2) |
| ![macOS](https://img.shields.io/badge/mac%20os-000000?style=for-the-badge&logo=apple&logoColor=white) | All Versions | ❌ Not Supported |

### Core Dependencies

*   **ROS 2 Humble**: The middleware backbone.
*   **Gazebo Classic (11)**: Default lightweight simulator.
*   **Python 3.10**: For the orchestrator and analysis tools.

---

## 🔌 Hardware Adaptation

If you are running benchmarks on a physical robot or specific hardware:

### Sensor Configuration
Ensure your URDF defines these frames:

*   `base_link`: Center of the robot
*   `odom`: Odometry frame
*   `scan`: 2D Lidar frame (if applicable)
*   `velodyne`: 3D Lidar frame (if applicable)

!!! tip "Verifying TF Tree"
    Run `ros2 run tf2_tools view_frames` to generate a PDF of your transformation tree and verify connectivity.
