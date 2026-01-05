# SLAM Auto-Tuner Guide

The Auto-Tuner is an advanced feature that uses Bayesian Optimization (via [Optuna](https://optuna.org/)) to automatically find the most accurate parameters for any SLAM algorithm.

## 🧠 How it Works
Instead of manually editing YAML files and running tests, the Auto-Tuner:
1.  Takes a **Base Job** as a template.
2.  Defines a **Search Space** (parameters and their ranges).
3.  Executes multiple **Trials**.
4.  Uses the **ATE (Absolute Trajectory Error)** as the objective to minimize.
5.  Suggests the **Best Parameters** found across all trials.

## 🛠 Usage via GUI

### Step 1: Prepare a Reference Job
The optimizer needs to know the context (World, Robot, SLAM stack). 
- Run any benchmark once from the Dashboard.
- Find its definition in `results/jobs/`.

### Step 2: Configure the Search Space
In **Tools > Auto-Tuner**:
1.  **Reference Job**: Browse to your generated job YAML.
2.  **Parameters**: Add parameters you want to optimize.
    - **Path**: The dot-separated path in the config (e.g., `slam.parameters.slam_gmapping.maxUrange`).
    - **Min/Max**: The range for the optimizer to explore.
3.  **Trials**: Set the number of experiments (recommended: 10-30).

### Step 3: Run & Analyze
Click **Run Optimization**. The orchestrator will run the simulations in the background. A final dialog will present the optimal values discovered.

---

## 💻 Technical Details

### Optimization Algorithm
We use the **Tree-structured Parzen Estimator (TPE)** algorithm. It is more efficient than a "Grid Search" because it learns which ranges correlate with better accuracy and focuses its search there.

### Process Isolation
Each trial is executed in a separate process. This ensures that:
- ROS 2 nodes are cleaned up between trials.
- Memory leaks don't accumulate.
- Failed trials (crashes) don't stop the whole optimization.

### Database
All results are stored in `results/optimization/<run_name>/study.db` (SQLite). You can open this file with any SQLite browser to see the history of every trial.

---

## ⚠️ Important Notes
- **Time Consumption**: Running 20 trials of a 5-minute benchmark will take over 1.5 hours. Ensure your PC is well-ventilated!
- **GPU Usage**: Optimization usually runs significantly faster on machines with dedicated GPUs.
- **Deterministic Results**: For better results, use a fixed seed if possible, or increase the number of trials to average out simulation noise.
