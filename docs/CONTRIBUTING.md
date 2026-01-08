# Contributing to BenchBot

Thank you for your interest in contributing to BenchBot! We welcome contributions from the community to make this the best ROS 2 benchmarking ecosystem.

This guide will help you get started with contributing code, documentation, or new features.

---

## 🛠️ Development Setup

### Prerequisites

- **OS**: Ubuntu 22.04 LTS (Jammy Jellyfish)
- **ROS 2**: Humble Hawksbill
- **Python**: 3.10+
- **Simulator**: Gazebo Classic or O3DE

### Installation

1.  **Fork and Clone** the repository:
    ```bash
    git clone https://github.com/YOUR_USERNAME/BenchBot.git
    cd BenchBot
    ```

2.  **Install Python Dependencies**:
    ```bash
    pip install -r requirements.txt
    ```

3.  **Install ROS 2 Dependencies**:
    ```bash
    # From the root of your workspace
    rosdep install --from-paths . --ignore-src -r -y
    ```

4.  **Verify Installation**:
    ```bash
    python3 -m pytest tests/
    ```

---

## 🚀 How to Contribute

### 1. Adding a New SLAM Algorithm

We designed BenchBot to be modular. To add a new SLAM:

1.  **Create a Config File**: Add `configs/slams/my_slam.yaml`.
    *   Define the launch command.
    *   Define the topics (map, scan, odom).
    *   Set up probes to detect when it's ready.

2.  **Test It**: Create a matrix `configs/matrices/test_my_slam.yaml` using your new SLAM ID.

3.  **Document It**: Update `docs/MULTI_SLAM_GUIDE.md` to list your new algorithm.

See [Multi-SLAM Guide](docs/MULTI_SLAM_GUIDE.md) for details.

### 2. Adding a New Metric

Metrics are located in `evaluation/metrics.py`.

1.  **Inherit** from `BaseMetric`.
2.  **Implement** the `compute(gt_map, slam_map)` method.
3.  **Register** your metric in the `METRICS_REGISTRY`.

```python
class MyNewMetric(BaseMetric):
    def compute(self, gt, slam):
        # Your math here
        return 0.95
```

### 3. Improving Documentation

Documentation is built with **MkDocs Material**.

1.  Edit files in `docs/`.
2.  Preview changes:
    ```bash
    mkdocs serve
    ```
3.  Open `http://127.0.0.1:8000` in your browser.

---

## 📏 Coding Standards

### Python

- We follow **PEP 8**.
- Use **Type Hints** for function arguments and return types.
- formatted with `black`.

```python
def calculate_trajectory(poses: List[Pose]) -> float:
    """Calculates the total length of a trajectory."""
    pass
```

### Git Commit Messages

- Use **Conventional Commits**:
    - `feat: add Hector SLAM support`
    - `fix: resolve crash in GMapping expiry`
    - `docs: translate FAQ to English`
    - `chore: update requirements.txt`

---

## 📥 Pull Request Process

1.  Create a new branch: `git checkout -b feat/my-new-feature`.
2.  Commit your changes.
3.  Push to your fork.
4.  Open a Pull Request (PR) against the `main` branch.
5.  Wait for the CI checks to pass and for a maintainer review.

---

## 💬 Community

- **Issues**: Use GitHub Issues for bug reports and feature requests.
- **Discussions**: Use GitHub Discussions for questions and ideas.

**Happy Coding!** 🚀
