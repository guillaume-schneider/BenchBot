# System Metrics Documentation

The SLAM Bench Orchestrator captures high-fidelity performance metrics to evaluate not only the quality of the SLAM map but also the computational efficiency of the algorithm.

## 📊 SLAM Quality Metrics

### 1. ATE (Absolute Trajectory Error)
- **Definition**: The Root Mean Square Error (RMSE) of the Euclidean distance between the estimated trajectory positions and the ground truth.
- **Formula**:

  $$
  \text{ATE} = \sqrt{\frac{1}{N} \sum_{i=1}^{N} \| \mathbf{p}_{est,i} - \mathbf{p}_{gt,i} \|^2}
  $$
  Where $N$ is the number of aligned poses, and $\mathbf{p}$ represents the $(x, y)$ position vector.
- **Alignment**: Trajectories are aligned using Umeyama's algorithm (rigid transformation optimization) before calculating error to account for start frame differences.
- **Goal**: Lower is better. ATE $< 0.1m$ is considered excellent for office environments.
- **Intuitively**: "On average, how far (in meters) is the robot's estimated position from its true position?" A low ATE means the robot knows exactly where it is.

### 2. Map Coverage
- **Definition**: The fraction of the **accessible ground truth free space** that has been successfully observed by the robot.
- **Formula**:

  $$
  \text{Coverage} = \frac{|GT_{free} \cap Est_{known}|}{|GT_{free}|}
  $$
  *   $GT_{free}$: Set of cells in Ground Truth that are free ($0$).
  *   $Est_{known}$: Set of cells in Estimated Map that are discovered (Free or Occupied, $\neq -1$).
- **Implementation**: Uses bitwise operations on the aligned occupancy grids.
- **Goal**: Higher is better. Measures exploration completeness.
- **Intuitively**: "Dis the robot find the whole room, or did it miss a corner?" 100% means it saw everything there was to see.

### 3. IoU (Intersection over Union)
- **Definition**: Measures the overlap consistency of **Occupied** cells (walls/obstacles) between the SLAM map and Ground Truth.
- **Formula**:

  $$
  \text{IoU} = \frac{|GT_{occ} \cap Est_{occ}|}{|GT_{occ} \cup Est_{occ}|}
  $$
  *   $GT_{occ}$: $\{c \in \text{Cells} \mid GT(c) > 50\}$
  *   $Est_{occ}$: $\{c \in \text{Cells} \mid Est(c) > 50\}$
- **Goal**: Higher is better. Only considers structural accuracy (walls). A low IoU usually indicates map distortion ("ghosting") or scale drift.
- **Intuitively**: "If you overlay the blueprint and the robot's map, do the walls overlap perfectly?" High IoU means walls are where they should be.

### 4. SSIM (Structural Similarity Index) 🆕
- **Definition**: A perceptual metric that quantifies image quality degradation caused by processing (compression or, in our case, mapping noise). 
- **Formula**:

  $$
  \text{SSIM}(x, y) = \frac{(2\mu_x\mu_y + C_1)(2\sigma_{xy} + C_2)}{(\mu_x^2 + \mu_y^2 + C_1)(\sigma_x^2 + \sigma_y^2 + C_2)}
  $$
  *   $\mu_x, \mu_y$: Local means of maps $x$ and $y$.
  *   $\sigma_x, \sigma_y$: Local variances.
  *   $\sigma_{xy}$: Local covariance.
- **Goal**: Higher is better (Target: $>0.8$). Captures "fuzziness" and structural coherence better than pixel-wise IoU.
- **Advantage**: Detects non-linear distortions (e.g., curved corridors) that IoU might miss.
- **Intuitively**: "Does the map look correct to a human eye?" SSIM punishes noise and fuzzy artifacts even if the walls are roughly in the right place.

### 5. Wall Thickness Analysis 🆕
- **Definition**: Estimates the average physical thickness of mapped walls to detect "blurring" caused by localization jitter.
- **Algorithm**:
  1.  **Skeletonize** the estimated map to find the 1-pixel wide centerlines of walls. $S = \text{Skeleton}(Est_{occ})$
  2.  Compute **Distance Transform** $D$ on the occupied mask (distance to nearest free cell).
  3.  Sample the distance map at skeleton locations:

  $$
  \text{Thickness} \approx 2 \times \text{mean}(D(p)) \quad \forall p \in S
  $$
- **Goal**: Lower is better. Values close to the actual wall thickness (~10cm) indicate a sharp, stable map. High values (>20cm) indicate that walls are "smeared" due to drift.
- **Intuitively**: "Are the walls sharp, thin lines, or thick, blurry blobs?" Sharp lines imply the robot's localization was stable; thick lines imply it was jittering around.

### 6. Path Length
- **What it is**: Total distance traveled by the robot.
- **Unit**: Meters (m).
- **Usage**: Used to normalize other metrics (e.g., drift per meter).

---

## 💻 System Performance Metrics (New)

These metrics are sampled at **1Hz** throughout the duration of the run, capturing the usage of the entire process tree (SLAM + Nav2 + Simulator).

### 7. Max CPU Usage
- **What it is**: The peak cumulative CPU usage across all cores.
- **Unit**: Percentage (%). Note: 100% means 1 full core at 100%. 
- **Critical Threshold**: If $> 100\%$ on a Raspberry Pi or similar single-board computer, the algorithm might be too heavy for edge deployment.

### 8. Max RAM Usage
- **What it is**: The peak Resident Set Size (RSS) memory consumed.
- **Unit**: Megabytes (MB).
- **Usage**: Essential for ensuring the stack fits within the hardware constraints of the robot.

---

## ⏱️ Temporal Metrics

### 9. Duration
- **What it is**: Total time elapsed between the start of movement and the end of the run.
- **Unit**: Seconds (s).
- **Usage**: Helps distinguish "slow and steady" algorithms from "fast and risky" ones.

---

## 🧠 Intelligent Anomaly Detection (Heuristics)

The orchestrator now includes a built-in "Brain" that evaluates if a run is a technical failure beyond simple completion.

### 10. Potential Failure Detection
A run is flagged with a **"POTENTIAL FAILURE"** badge if any of the following are detected:
- **Major TF Jumps**: If the robot "teleports" (calculated speed pulses $> 1.5 m/s$).
- **Stuck Robot**: If the robot traveled less than $0.2m$ despite the process running.
- **Massive Drift**: If the ATE RMSE exceeds $1.0m$.
- **No Data**: If the rosbag is empty or missing key topics (odometry/scan).
