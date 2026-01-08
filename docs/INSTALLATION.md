---
icon: material/download
---

# 📦 Installation Guide

Choose your preferred installation method below. We support both a **fully automated** script and a **step-by-step manual** process.

## 📋 Prerequisites Checklist

Ensure your system meets these requirements before proceeding:

- [ ] **OS**: Ubuntu 22.04 LTS (Jammy Jellyfish)
- [ ] **ROS 2**: Humble Hawksbill installed ([Official Guide](https://docs.ros.org/en/humble/Installation.html))
- [ ] **Python**: Version 3.10 or higher
- [ ] **Git**: Installed (`sudo apt install git`)

---

## 🛠️ Installation Methods

=== "✨ Automated (Recommended)"

    The easiest way to get started. This script configures workspace, dependencies, and environments for you.

    ```bash
    # 1. Clone
    git clone https://github.com/guillaume-schneider/BenchBot.git
    cd BenchBot

    # 2. Run Installer
    chmod +x install.sh
    ./install.sh
    ```

    !!! success "What does `install.sh` do?"
        *   Creates a python virtual environment `.venv`
        *   Installs system packages via `apt`
        *   Installs Python dependencies via `pip`
        *   Builds ROS 2 workspace

=== "🔧 Manual Setup"

    If you prefer full control over your environment.

    **1. Clone the repo**
    ```bash
    git clone https://github.com/guillaume-schneider/BenchBot.git
    cd BenchBot
    ```

    **2. System Dependencies**
    ```bash
    sudo apt update
    sudo apt install -y python3-venv python3-pip ros-humble-desktop
    ```

    **3. Python Environment**
    ```bash
    python3 -m venv .venv
    source .venv/bin/activate
    pip install -r requirements.txt
    ```

=== "🐳 Docker (Isolated)"

    Run everything in a container. No system pollution.

    ```bash
    # Build image
    docker-compose build

    # Run
    docker-compose up
    ```
    
    See the [Docker Guide](README.md#docker-mode) for more details.

---

## ✅ Verify Installation

Run the health check tool to confirm everything is ready:

```bash
python3 tools/health_check.py
```

!!! info "Expected Output"
    You should see green checkmarks `[✓]` for ROS 2, Python, and Simulator definitions.

## 🆘 Troubleshooting

*   **Missing Dependencies?** Run `pip install -r requirements.txt` again.
*   **ROS 2 not found?** Ensure you ran `source /opt/ros/humble/setup.bash`.
*   **Permission Denied?** Check `chmod +x install.sh`.

See [Troubleshooting](TROUBLESHOOTING_EXPLORATION.md) for more.
