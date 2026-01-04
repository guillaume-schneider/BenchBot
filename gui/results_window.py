import sys
from pathlib import Path
import yaml
import numpy as np
from PyQt5.QtWidgets import (QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, 
                             QLabel, QTextEdit, QPushButton, QMessageBox)
from PyQt5.QtCore import Qt

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
        compute_iou,
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
        compute_iou,
        compute_time_to_coverage,
        compute_path_length,
    )

class ResultWindow(QMainWindow):
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
        header = QLabel(f"Results for: {self.run_dir.name}")
        header.setStyleSheet("font-size: 18px; font-weight: bold; color: #f8fafc; margin-bottom: 10px;")
        layout.addWidget(header)
        
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
            
            # Need last map msg for accessible coverage
            _, last_map_msg = map_msgs[-1]
            
            cov = compute_coverage(gt_map, est_map)
            
            acc_cov = compute_accessible_coverage(
                gt_map, est_map, gt_res, gt_origin,
                (last_map_msg.info.origin.position.x, last_map_msg.info.origin.position.y),
                last_map_msg.info.width, last_map_msg.info.height, last_map_msg.info.resolution
            )
            
            iou = compute_iou(gt_map, est_map)
            path_len = compute_path_length(odom_msgs)
            
            self.log(f"\n--- RESULTS ---")
            self.log(f"Coverage: {cov*100:.2f}%")
            self.log(f"Accessible Coverage: {acc_cov*100:.2f}%")
            self.log(f"IoU: {iou:.4f}")
            self.log(f"Path Length: {path_len:.2f} m")
            
            # Plot
            self.ax_gt.clear()
            self.ax_est.clear()
            self.ax_gt.set_title("Ground Truth", color='white')
            self.ax_est.set_title("Estimated Map", color='white')
            
            self.display_map(self.ax_gt, gt_map)
            self.display_map(self.ax_est, est_map)
            
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
