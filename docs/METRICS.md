---
icon: material/chart-bell-curve-cumulative
---

# 📊 Metrics Explained

BenchBot calculates 5 key metrics to evaluate SLAM quality. Understanding them is crucial for analysis.

## 🎯 1. ATE (Absolute Trajectory Error)

Measures **localization accuracy**: how far is the robot's estimated path from the ground truth?

!!! info "Formula"
    $$ \text{ATE} = \sqrt{\frac{1}{N} \sum_{i=1}^{N} \| \mathbf{p}_{est,i} - \mathbf{p}_{gt,i} \|^2} $$
    
    *Aligned using Umeyama algorithm (rigid transformation).*

| Value | Interpretation | Rating |
| :--- | :--- | :--- |
| **< 0.05m** | Extremely Precise (5cm) | ⭐⭐⭐⭐⭐ |
| **< 0.15m** | Good for Indoor Nav | ⭐⭐⭐⭐ |
| **> 0.50m** | Localization Failure | ❌ |

---

## 🗺️ 2. Map Quality (SSIM & IoU)

Measures **mapping fidelity**: does the generated map look like the real world?

### SSIM (Structural Similarity)
Compares geometric structure (corners, walls) rather than just pixel overlap.

| Value | Interpretation |
| :--- | :--- |
| **1.0** | Perfect Replica |
| **> 0.85** | High Fidelity |
| **< 0.60** | Distorted Map |

### IoU (Intersection over Union)
$$ \text{IoU} = \frac{\text{Area of Overlap}}{\text{Area of Union}} $$

---

## 🧱 3. Wall Thickness

Detects **map blur** caused by sensor noise or drift. A sharp map has thin walls.

*   **Ideal Wall**: ~5-10cm (depends on Lidar resolution)
*   **Blurred Wall**: > 20cm (indicates "Double Wall" effect)

---

## 🖥️ 4. System Resources

| Metric | Description | Goal |
| :--- | :--- | :--- |
| **CPU Usage** | Normalized % across all cores. | Minimize (< 50%) |
| **RAM Usage** | Peak memory consumption in MB. | Minimize (< 2GB) |

!!! tip "Performance vs Accuracy"
    Higher particle counts increase **Accuracy** (lower ATE) but drastically increase **CPU Usage**. Use the Auto-Tuner to find the sweet spot.
