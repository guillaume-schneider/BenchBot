---
icon: material/rocket-launch
---

# ⚡ Quick Start

Get your first benchmark running in less than **60 seconds**.

!!! tip "Prerequisites"
    Ensure you have **Ubuntu 22.04**, **ROS 2 Humble**, and **Python 3.10+**.

## 🚀 The " 3-Command " Start

Copy and paste this entire block into your terminal:

```bash
# 1. Clone & Install
git clone https://github.com/guillaume-schneider/BenchBot.git
cd benchbot && ./install.sh

# 2. Launch Interface
python3 gui/main.py
```

---

## 🎮 Your First Run

Once the GUI opens, follow these 3 steps:

1.  **Select Matrix**: Go to `Dashboard` → select `test_minimal.yaml`.
2.  **Launch**: Click the big blue **RUN BENCHMARK** button.
3.  **Watch**: See the real-time metrics and robot view appear.

!!! success "What just happened?"
    You just ran a complete SLAM benchmark! The system automatically:
    
    *   spun up a **Docker container** or local ROS nodes.
    *   launched **Gazebo** with a simulated environment.
    *   recorded metrics (ATE, CPU, RAM).
    *   generated a **PDF Report** in `results/`.

---

## ⏭️ What's Next?

<div class="grid cards" markdown>

-   :material-cog: __[Configuration](CONFIG_REFERENCE.md)__
    Customize your SLAM parameters

-   :material-chart-bell-curve-cumulative: __[Understand Metrics](METRICS.md)__
    Learn what ATE and SSIM mean

-   :material-robot: __[Simulators](SIMULATORS.md)__
    Switch to Photorealistic O3DE

</div>
