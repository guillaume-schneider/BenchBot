---
icon: material/gamepad-variant
---

# 🎮 Simulators

BenchBot supports two simulation backends to balance between performance and realism.

## 🆚 Comparison

| Feature | Gazebo Classic (11) | O3DE (Open 3D Engine) |
| :--- | :--- | :--- |
| **Realism** | ⭐⭐ | ⭐⭐⭐⭐⭐ |
| **Physics** | ODE (Basic) | NVIDIA PhysX 5.0 |
| **Rendering** | OGRE (Simple) | Atom (Raytracing) |
| **Performance** | Fast (Low CPU) | Heavy (Requires GPU) |
| **Use Case** | Quick Logic Testing | Visual Demos & Sim-to-Real |

---

## 🛠️ Setup Guide

!!! tip "⚡ Automated Installation"
    Both Gazebo and O3DE can be installed and configured directly from the **BenchBot GUI**.
    Go to `Settings` → `Simulators` and click **Install/Repair** to let the system handle the dependencies for you.

=== "🦁 Gazebo Classic"

    **Default simulator.** Best for CI/CD and rapid development.

    ### Installation
    Gazebo 11 comes pre-installed with `ros-humble-desktop-full`.
    
    To verify:
    ```bash
    gazebo --version
    ```

    ### Running
    BenchBot uses Gazebo by default (`simulator: gazebo` in config).
    
    ```bash
    # Test a world manually
    ros2 launch benchbot_simulation gazebo_world.launch.py world:=warehouse
    ```

=== "⚛️ O3DE (Photorealistic)"

    **Advanced simulator.** Requires correct setup of the ROS 2 Gem.

    ### Prerequisites
    - **NVIDIA GPU** (RTX 2060 or higher recommended)
    - **Storage**: ~40GB required for engine + assets

    ### Setup Steps
    1.  **Download O3DE**: Get the Linux installer from [o3de.org](https://o3de.org).
    2.  **Install ROS 2 Gem**:
        ```bash
        cd <O3DE_PATH>
        ./scripts/o3de.sh register --gem-path <BENCHBOT_PATH>/sim/o3de_gem
        ```
    3.  **Build Project**:
        ```bash
        <O3DE_PATH>/cmake/cmake-build-linux.sh -p <PROJECT_PATH>
        ```

    !!! tip "Headless O3DE"
        You can run O3DE without a window using the `-r Null` flag in `HEADLESS_CI` mode, but it still requires a GPU context.

---

## 🌍 Supported Worlds

<div class="grid cards" markdown>

-   __Warehouse__
    Large industrial space with shelves and moving forklifts.
    *(Best for: AMCL, Coverage)*

-   __House__
    Domestic environment with narrow doors and furniture.
    *(Best for: Path Planning)*

-   __Office__
    Complex loops and glass walls (O3DE only).
    *(Best for: Loop Closure, Noise robustness)*

</div>
