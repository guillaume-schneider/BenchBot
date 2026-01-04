from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton, 
    QTabWidget, QFileDialog, QFormLayout, QLineEdit, QTextEdit, QSplitter, QFrame, QMessageBox
)
from PyQt5.QtCore import Qt
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
import numpy as np
from pathlib import Path

# Imports from project
from gui.pages.gt_map import GTMapPage
from evaluation.metrics import (
    load_gt_map, read_messages_by_topic, occupancy_arrays_from_msgs,
    compute_coverage, compute_iou, compute_path_length, compute_time_to_coverage
)

class ManualAnalysisPage(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.layout = QVBoxLayout(self)
        self.layout.setContentsMargins(20, 20, 20, 20)
        self.init_ui()
        
    def init_ui(self):
        # Header
        header = QLabel("Manual Run Analysis")
        header.setObjectName("headerLabel")
        self.layout.addWidget(header)
        
        # Inputs Card
        input_card = QFrame()
        input_card.setProperty("class", "card")
        form_layout = QFormLayout(input_card)
        
        # Bag Input
        bag_row = QHBoxLayout()
        self.bag_path = QLineEdit()
        self.bag_path.setPlaceholderText("Select rosbag folder (e.g. results/runs/.../bags/output)")
        bag_browse = QPushButton("Browse")
        bag_browse.clicked.connect(self.browse_bag)
        bag_row.addWidget(self.bag_path)
        bag_row.addWidget(bag_browse)
        form_layout.addRow("Rosbag Folder:", bag_row)
        
        # GT Input
        gt_row = QHBoxLayout()
        self.gt_path = QLineEdit()
        self.gt_path.setPlaceholderText("Select GT Map YAML")
        gt_browse = QPushButton("Browse")
        gt_browse.clicked.connect(self.browse_gt)
        gt_row.addWidget(self.gt_path)
        gt_row.addWidget(gt_browse)
        form_layout.addRow("GT Map (YAML):", gt_row)
        
        self.layout.addWidget(input_card)
        
        # Action
        ana_btn = QPushButton("Run Analysis")
        ana_btn.setObjectName("actionButton")
        ana_btn.clicked.connect(self.run_analysis)
        self.layout.addWidget(ana_btn)
        
        # Results Splitter
        splitter = QSplitter(Qt.Horizontal)
        
        # Log Area
        self.log_text = QTextEdit()
        self.log_text.setReadOnly(True)
        self.log_text.setStyleSheet("font-family: monospace;")
        splitter.addWidget(self.log_text)
        
        # Plot Area
        plot_container = QWidget()
        plot_layout = QVBoxLayout(plot_container)
        plot_layout.setContentsMargins(0,0,0,0)
        
        self.figure = Figure(figsize=(5, 4), dpi=100)
        self.canvas = FigureCanvas(self.figure)
        plot_layout.addWidget(self.canvas)
        
        splitter.addWidget(plot_container)
        splitter.setStretchFactor(0, 1)
        splitter.setStretchFactor(1, 2)
        
        self.layout.addWidget(splitter)
        
    def browse_bag(self):
        d = QFileDialog.getExistingDirectory(self, "Select Rosbag Folder")
        if d: self.bag_path.setText(d)
        
    def browse_gt(self):
        f, _ = QFileDialog.getOpenFileName(self, "Select GT Map", "", "YAML (*.yaml)")
        if f: self.gt_path.setText(f)
        
    def log(self, msg):
        self.log_text.append(msg)
        
    def run_analysis(self):
        bag = self.bag_path.text().strip()
        gt = self.gt_path.text().strip()
        
        if not bag or not gt:
            QMessageBox.warning(self, "Missing Input", "Please select both rosbag folder and GT map.")
            return
            
        self.log_text.clear()
        self.log(f"Analyzing...\nBag: {bag}\nGT: {gt}\n")
        self.figure.clear()
        
        try:
            # 1. Load GT
            gt_map, gt_res, gt_origin = load_gt_map(gt)
            self.log(f"GT Loaded: shape={gt_map.shape}, res={gt_res}")
            
            # 2. Read Bag
            topics = ["/map", "/odom"]
            data = read_messages_by_topic(bag, topics)
            map_msgs = data.get("/map", [])
            odom_msgs = data.get("/odom", [])
            
            if not map_msgs:
                self.log("❌ Error: No /map messages found.")
                return
                
            # 3. Compute Metrics
            # Note: occupancy_arrays_from_msgs returns (est_map, last_map_msg) tuple now
            est_map, last_msg = occupancy_arrays_from_msgs(map_msgs, gt_map, gt_res, gt_origin)
            
            final_cov = compute_coverage(gt_map, est_map)
            final_iou = compute_iou(gt_map, est_map)
            path_len = compute_path_length(odom_msgs)
            
            self.log(f"Final Coverage: {final_cov*100:.2f}%")
            self.log(f"IoU: {final_iou:.4f}")
            self.log(f"Path Length: {path_len:.2f} m")
            
            # Time to coverage (optional)
            times = compute_time_to_coverage(gt_map, gt_res, gt_origin, map_msgs, [0.5, 0.8, 0.9])
            t0 = map_msgs[0][0]
            for th, t in times.items():
                if t:
                    self.log(f"Time to {int(th*100)}%: {t-t0:.2f}s")
                else:
                    self.log(f"Time to {int(th*100)}%: Not reached")
            
            # 4. Plot
            ax1 = self.figure.add_subplot(121)
            ax2 = self.figure.add_subplot(122)
            
            ax1.set_title("Ground Truth")
            ax1.imshow(self._to_vis(gt_map), cmap="gray", vmin=0, vmax=255)
            ax1.axis('off')
            
            ax2.set_title("Estimated")
            ax2.imshow(self._to_vis(est_map), cmap="gray", vmin=0, vmax=255)
            ax2.axis('off')
            
            self.canvas.draw()
            self.log("\n✅ Analysis Complete.")
            
        except Exception as e:
            self.log(f"\n❌ Error: {e}")
            import traceback
            self.log(traceback.format_exc())

    def _to_vis(self, grid):
        vis = np.zeros_like(grid, dtype=np.uint8)
        vis[grid == -1] = 127 # Unknown
        vis[grid == 0] = 255  # Free
        vis[grid > 50] = 0    # Occupied
        return vis

class ToolsPage(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        
        self.tabs = QTabWidget()
        self.tabs.setStyleSheet("""
            QTabWidget::pane { border: none; }
            QTabBar::tab { background: transparent; color: #94a3b8; padding: 10px 20px; font-weight: 600; }
            QTabBar::tab:selected { color: #818cf8; border-bottom: 2px solid #818cf8; }
        """)
        
        self.gt_page = GTMapPage()
        self.manual_page = ManualAnalysisPage()
        
        self.tabs.addTab(self.gt_page, "GT Generator")
        self.tabs.addTab(self.manual_page, "Manual Analysis")
        
        layout.addWidget(self.tabs)
