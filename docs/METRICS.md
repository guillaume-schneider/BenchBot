# System Metrics Documentation

The SLAM Bench Orchestrator captures high-fidelity performance metrics to evaluate not only the quality of the SLAM map but also the computational efficiency of the algorithm.

## 📊 SLAM Quality Metrics

### 1. ATE (Absolute Trajectory Error)
- **What it is**: Root Mean Square Error (RMSE) between the ground truth trajectory and the estimated trajectory.
- **Unit**: Meters (m).
- **Goal**: Lower is better. ATE $< 0.1m$ is considered excellent for office environments.

### 2. Map Coverage
- **What it is**: Percentage of the explorable ground truth area that has been discovered (marked as free or occupied) by the SLAM algorithm.
- **Unit**: Percentage (%).
- **Goal**: Higher is better. This measures the efficiency of the exploration strategy combined with the SLAM's ability to "see" and map space.

### 3. IoU (Intersection over Union)
- **What it is**: Measures the alignment between the occupancy grid of the SLAM and the Ground Truth.
- **Goal**: Higher is better. A low IoU despite high coverage indicates significant "ghosting" or map drift/stretching.

### 4. Path Length
- **What it is**: Total distance traveled by the robot.
- **Unit**: Meters (m).
- **Usage**: Used to normalize other metrics (e.g., drift per meter).

---

## 💻 System Performance Metrics (New)

These metrics are sampled at **1Hz** throughout the duration of the run, capturing the usage of the entire process tree (SLAM + Nav2 + Simulator).

### 5. Max CPU Usage
- **What it is**: The peak cumulative CPU usage across all cores.
- **Unit**: Percentage (%). Note: 100% means 1 full core at 100%. 
- **Critical Threshold**: If $> 100\%$ on a Raspberry Pi or similar single-board computer, the algorithm might be too heavy for edge deployment.

### 6. Max RAM Usage
- **What it is**: The peak Resident Set Size (RSS) memory consumed.
- **Unit**: Megabytes (MB).
- **Usage**: Essential for ensuring the stack fits within the hardware constraints of the robot.

---

## ⏱️ Temporal Metrics

### 7. Duration
- **What it is**: Total time elapsed between the start of movement and the end of the run.
- **Unit**: Seconds (s).
- **Usage**: Helps distinguish "slow and steady" algorithms from "fast and risky" ones.
