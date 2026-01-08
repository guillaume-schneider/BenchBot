# Analysis & Comparison Guide 📊

The Comparison tool in SLAM Bench Orchestrator allows for deep analytical dives into how different SLAM algorithms perform under the same conditions.

## ⚖️ Comparing Multiple Runs

1.  **Selection**: Use the side-by-side dropdowns to select up to **3 completed runs**.
2.  **Comparison Table**: 
    *   Compare **ATE (RMSE)**, **Coverage**, **RAM**, and **CPU** in a single view.
    *   **Health Status**: Check the top row for a quick "✅ VALID" or "❌ ANOMALY" status.
    *   **Tooltips**: Hover over "ANOMALY" to see exactly what went wrong (TF jumps, drift, etc.).
3.  **Trajectory Overlay**:
    *   All selected trajectories are plotted on the same coordinate system.
    *   **Background Map**: If the runs share a common environment, the Ground Truth map is automatically loaded and aligned in the background.

## 🧠 Behind the Scenes: Heuristic Anomaly Detection

To save time during large-scale benchmarking, the orchestrator automatically "grades" the validity of a run using heuristics:

| Anomaly Type | Threshold | Meaning |
| :--- | :--- | :--- |
| **TF Jump** | Speed $> 1.5 m/s$ | SLAM likely "teleposted" the robot due to internal pose correction failure. |
| **Low Movement**| Dist $< 0.2 m$ | The robot didn't move. Navigation or Simulator process might have crashed. |
| **Massive Drift** | ATE $> 1.0 m$ | The SLAM estimate is completely detached from reality. |

## 📈 Tips for Analysis

*   **Drift per Meter**: Look at the relation between **Path Length** and **ATE**. A long run with low ATE is more impressive than a short one.
*   **CPU vs. Precision**: Use the Comparison table to see if the higher precision of a specific SLAM (like Cartographer) justifies its higher CPU cost compared to simpler ones (like GMapping).
*   **Visual Alignment**: If the trajectories don't align well with the background Ground Truth walls, it's a visual indicator of **Absolute Trajectory Error**.
