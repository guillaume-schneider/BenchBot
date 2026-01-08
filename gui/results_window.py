"""Results window for displaying benchmark run analysis.

Provides a detailed view of benchmark results including:
- Metrics computation (Coverage, IoU, SSIM, ATE)
- Map visualization (Ground Truth vs Estimated)
- Optimization suggestions
"""
import sys
from pathlib import Path
import yaml
import numpy as np
from PyQt5.QtWidgets import (QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, 
                             QLabel, QTextEdit, QPushButton, QMessageBox)
from PyQt5.QtCore import Qt, pyqtSignal

from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg
from matplotlib.figure import Figure
import matplotlib.pyplot as plt

# Attempt to import metrics
try:
    from evaluation.metrics import (
        load_gt_map,
        read_messages_by_topic,
        occupancy_arrays_from_msgs,
        compute_coverage,
        compute_accessible_coverage,
        save_map_image,
        compute_iou,
        compute_ssim,
        compute_wall_thickness,
        compute_time_to_coverage,
        compute_path_length,
    )
except ImportError:
    # Handle case where evaluation package is not in pythonpath correctly
    import sys
    sys.path.append(str(Path(__file__).parent.parent))
    from evaluation.metrics import (
        load_gt_map,
        read_messages_by_topic,
        occupancy_arrays_from_msgs,
        compute_coverage,
        compute_accessible_coverage,
        save_map_image,
        compute_iou,
        compute_ssim,
        compute_wall_thickness,
        compute_time_to_coverage,
        compute_path_length,
    )

class ResultWindow(QMainWindow):
    """Window for displaying detailed benchmark run results.
    
    Automatically loads and analyzes benchmark results, computing metrics
    and displaying map comparisons.
    
    Signals:
        auto_tune_requested: Emitted when user requests optimization (str: job_path)
    
    Args:
        run_dir: Path to the run directory containing results
        parent: Optional parent widget
    """
    auto_tune_requested = pyqtSignal(str) # job_path

    def __init__(self, run_dir, parent=None):
        super().__init__(parent)
        self.run_dir = Path(run_dir)
        self.setWindowTitle(f"Run Results: {self.run_dir.name}")
        self.resize(1100, 800)
        self.setStyleSheet("QMainWindow { background-color: #0f172a; color: #f8fafc; }")
        
        # Central Widget
        central = QWidget()
        self.setCentralWidget(central)
        layout = QVBoxLayout(central)
        
        # Header
        header_layout = QHBoxLayout()
        header = QLabel(f"Results for: {self.run_dir.name}")
        header.setStyleSheet("font-size: 18px; font-weight: bold; color: #f8fafc;")
        header_layout.addWidget(header)
        
        header_layout.addStretch()
        
        self.btn_tune = QPushButton("Optimize this Run")
        self.btn_tune.setStyleSheet("""
            QPushButton { 
                background-color: #6366f1; color: white; padding: 6px 15px; 
                border-radius: 6px; font-weight: bold; font-size: 13px;
            }
            QPushButton:hover { background-color: #4f46e5; }
        """)
        self.btn_tune.clicked.connect(self.on_tune_clicked)
        header_layout.addWidget(self.btn_tune)
        
        layout.addLayout(header_layout)
        
        # Content Layout (Text left, Plots right)
        content_layout = QHBoxLayout()
        
        # Text Log
        self.log_view = QTextEdit()
        self.log_view.setReadOnly(True)
        self.log_view.setStyleSheet("""
            QTextEdit { background-color: #1e293b; color: #cbd5e1; border: 1px solid #334155; border-radius: 8px; font-family: Monospace; font-size: 12px; padding: 10px; }
        """)
        self.log_view.setFixedWidth(400)
        content_layout.addWidget(self.log_view)
        
        # Plots
        plot_container = QWidget()
        plot_layout = QVBoxLayout(plot_container)
        plot_layout.setContentsMargins(0,0,0,0)
        
        self.figure = Figure(figsize=(8, 6), facecolor='#0f172a')
        self.canvas = FigureCanvasQTAgg(self.figure)
        plot_layout.addWidget(self.canvas)
        
        self.ax_gt = self.figure.add_subplot(121)
        self.ax_est = self.figure.add_subplot(122)
        
        # Style axes
        for ax in [self.ax_gt, self.ax_est]:
            ax.set_facecolor('#0f172a')
            ax.tick_params(colors='white')
            ax.xaxis.label.set_color('white')
            ax.yaxis.label.set_color('white')
            ax.spines['bottom'].set_color('#334155')
            ax.spines['top'].set_color('#334155') 
            ax.spines['left'].set_color('#334155')
            ax.spines['right'].set_color('#334155')

        content_layout.addWidget(plot_container)
        layout.addLayout(content_layout)
        
        # Run analysis immediately
        self.run_analysis()
        
    def log(self, msg):
        self.log_view.append(msg)
        print(f"RESULT_WIN: {msg}")

    def on_tune_clicked(self):
        config_path = self.run_dir / "config_resolved.yaml"
        if config_path.exists():
            self.auto_tune_requested.emit(str(config_path))
        else:
            QMessageBox.warning(self, "Error", "Could not find reference configuration for this run.")

    def run_analysis(self):
        try:
            self.log("Loading configuration...")
            config_path = self.run_dir / "config_resolved.yaml"
            if not config_path.exists():
                self.log(f"ERROR: Config not found at {config_path}")
                return
                
            with open(config_path, 'r') as f:
                config = yaml.safe_load(f)
                
            # Extract GT Path
            # dataset -> ground_truth -> map_path
            ds = config.get("dataset", {})
            gt_info = ds.get("ground_truth", {})
            gt_path_rel = gt_info.get("map_path")
            
            if not gt_path_rel:
                self.log("ERROR: No ground_truth path in config.")
                return
                
            # Resolve GT path (relative to project root usually)
            # config_resolved is in results/runs/RUN_ID/
            # map_path is usually maps/gt/model.yaml (relative to root)
            # We assume we run from root.
            project_root = Path.cwd() # safe assumption?
            gt_path = project_root / gt_path_rel
            
            if not gt_path.exists():
                # Try relative to run dir?
                gt_path = self.run_dir / gt_path_rel
                
            if not gt_path.exists():
                self.log(f"ERROR: GT Map not found at {gt_path}")
                return
                
            self.log(f"Loading GT Map: {gt_path.name}")
            gt_map, gt_res, gt_origin = load_gt_map(str(gt_path))
            self.log(f"GT Loaded. Shape: {gt_map.shape}")
            
            # Find Bag
            # Usually in bags/
            bag_dir = self.run_dir / "bags"
            if not bag_dir.exists():
                 # checking root
                 bag_dir = self.run_dir
            
            # Find any .db3 file recursively or folder
            # ROS2 bags are folders.
            # Look for subdirs in bags/
            candidates = [p for p in bag_dir.glob("**/*.db3")]
            if not candidates:
                 self.log("ERROR: No .db3 files found in run dir.")
                 return
            
            # Use the folder containing the db3
            bag_path = candidates[0].parent
            self.log(f"Reading Bag: {bag_path}")
            
            topics = ["/map", "/odom"]
            msgs_by_topic = read_messages_by_topic(str(bag_path), topics)
            
            map_msgs = msgs_by_topic.get("/map", [])
            odom_msgs = msgs_by_topic.get("/odom", [])
            
            if not map_msgs:
                self.log("WARNING: No /map messages.")
            else:
                self.log(f"Found {len(map_msgs)} map messages.")
                
            # Metrics
            est_map = occupancy_arrays_from_msgs(map_msgs, gt_map, gt_res, gt_origin)
            
            # Save Maps for report
            bag_path_obj = Path(bag_path)
            map_img_path = str(bag_path_obj / "final_map.png")
            gt_img_path = str(bag_path_obj / "gt_map.png")
            
            try:
                self.log(f"Saving maps to {map_img_path} and {gt_img_path}")
                save_map_image(est_map, map_img_path, title="Estimated Map")
                save_map_image(gt_map, gt_img_path, title="Ground Truth Map")
            except Exception as e:
                self.log(f"WARNING: Could not save map images: {e}")
            
            
            # Need last map msg for accessible coverage
            _, last_map_msg = map_msgs[-1]
            
            cov = compute_coverage(gt_map, est_map)
            
            acc_cov = compute_accessible_coverage(
                gt_map, est_map, gt_res, gt_origin,
                (last_map_msg.info.origin.position.x, last_map_msg.info.origin.position.y),
                last_map_msg.info.width, last_map_msg.info.height, last_map_msg.info.resolution
            )
            
            iou = compute_iou(gt_map, est_map)
            ssim_val = compute_ssim(gt_map, est_map)
            wall_thick_m = compute_wall_thickness(est_map, gt_res)
            
            path_len = compute_path_length(odom_msgs)
            
            # RMSE Calculation
            rmse = None
            try:
                from tools.benchmark import run_benchmark
                # run_benchmark already handles plotting a separate ate_plot.png
                rmse = run_benchmark(str(bag_path))
            except Exception as e:
                self.log(f"WARNING: RMSE calculation failed: {e}")
            
            # Compute Duration
            duration_s = 0.0
            if odom_msgs:
                t_start = odom_msgs[0][0]
                t_end = odom_msgs[-1][0]
                duration_s = t_end - t_start
            
            self.log(f"\n--- RESULTS ---")
            self.log(f"Duration: {duration_s:.2f} s")
            self.log(f"Coverage: {cov*100:.2f}%")
            self.log(f"Accessible Coverage: {acc_cov*100:.2f}%")
            self.log(f"IoU: {iou:.4f}")
            self.log(f"Path Length: {path_len:.2f} m")
            
            # Save metrics to json
            metrics_file = self.run_dir / "metrics.json"
            data = {}
            if metrics_file.exists():
                import json
                with open(metrics_file, 'r') as f:
                    try:
                        data = json.load(f)
                    except:
                        pass
            
            # Only update metrics that don't already exist (preserve orchestrator data)
            if data.get("duration_s") is None:
                data["duration_s"] = float(duration_s)
            if data.get("coverage") is None:
                data["coverage"] = float(cov)
            if data.get("accessible_coverage") is None:
                data["accessible_coverage"] = float(acc_cov)
            if data.get("iou") is None:
                data["iou"] = float(iou)
            if data.get("occupancy_iou") is None:
                data["occupancy_iou"] = float(iou)
            if data.get("map_ssim") is None:
                data["map_ssim"] = float(ssim_val)
            if data.get("wall_thickness_m") is None:
                data["wall_thickness_m"] = float(wall_thick_m)
            if data.get("path_length_m") is None:
                data["path_length_m"] = float(path_len)
            # Add map image paths
            if data.get("map_image_path") is None:
                data["map_image_path"] = map_img_path
            if data.get("gt_map_image_path") is None:
                data["gt_map_image_path"] = gt_img_path
            if rmse is not None and data.get("ate_rmse") is None:
                data["ate_rmse"] = float(rmse)
            
            # Legacy fallbacks (only if not present)
            if data.get("coverage_percent") is None:
                data["coverage_percent"] = float(cov * 100)
            if data.get("accessible_coverage_percent") is None:
                data["accessible_coverage_percent"] = float(acc_cov * 100)
            
            import json
            with open(metrics_file, 'w') as f:
                json.dump(data, f, indent=4)
            self.log(f"Metrics saved to {metrics_file.name}")
            
            # Plot
            self.ax_gt.clear()
            self.ax_est.clear()
            self.ax_gt.set_title("Ground Truth", color='white')
            self.ax_est.set_title("Estimated Map", color='white')
            

            self.display_map(self.ax_gt, np.flipud(gt_map))
            # Flip estimated map to match GT orientation
            self.display_map(self.ax_est, np.flipud(est_map))
            
            self.canvas.draw()
            
        except Exception as e:
            self.log(f"CRITICAL ERROR: {e}")
            import traceback
            traceback.print_exc()

    def display_map(self, ax, grid):
        # -1 -> 128, 0 -> 255, 100 -> 0
        vis = np.zeros_like(grid, dtype=np.uint8)
        vis[grid == -1] = 127
        vis[grid == 0] = 255
        vis[grid > 50] = 0
        
        ax.imshow(vis, cmap='gray', vmin=0, vmax=255)
        ax.axis('off')
