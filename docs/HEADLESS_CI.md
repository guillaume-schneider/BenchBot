# Headless CI / CLI Guide

The SLAM Bench Orchestrator is designed to run in continuous integration (CI) environments where no graphical interface is available.

## 🛠️ CLI Runners

### 1. Running a Full Matrix (`run_matrix.py`)
This is the preferred way to run benchmark suites on a server. It resolves all configurations and executes jobs sequentially.

```bash
python3 runner/run_matrix.py configs/matrices/test_headless_ci.yaml
```

### 2. Running a Single Job (`run_one.py`)
If you have a `config_resolved.yaml` (automatically generated in `results/jobs/`), you can run it directly:

```bash
python3 runner/run_one.py results/jobs/my_job.yaml
```

---

## 🖥️ Headless Configuration

To ensure a run is fully headless, your matrix configuration must pass specific arguments to simulation processes.

### Example Dataset Config
In your matrix `datasets` or `scenario` section:

```yaml
- name: nav2_sim
  cmd:
    - ros2
    - launch
    - ...
    - use_gazebo:=False  # Disables Gazebo GUI
    - use_rviz:=False    # Disables RViz
```

---

## 📋 Integration in CI (GitHub Actions)

Example workflow snippet:

```yaml
jobs:
  benchmark:
    runs-on: ubuntu-latest
    container: ros:humble
    steps:
      - uses: actions/checkout@v2
      - name: Install dependencies
        run: pip install -r requirements.txt
      - name: Run Headless Benchmark
        run: python3 runner/run_matrix.py configs/matrices/test_headless_ci.yaml
      - name: Archive Results
        uses: actions/upload-artifact@v2
        with:
          path: results/runs_ci/
```

## 🔍 Result Verification
Even without a GUI, you can check the results in:
1. `results/index_ci.jsonl`: One line per run with summary metrics.
2. `results/runs_ci/<RUN_ID>/metrics.json`: Detailed dictionary of all calculated metrics.
