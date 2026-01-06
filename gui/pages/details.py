from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton, QTabWidget, 
    QTextEdit, QGroupBox, QComboBox, QSplitter, QFrame, QFormLayout,
    QTableWidget, QTableWidgetItem, QHeaderView
)
import json
import datetime
from pathlib import Path
from PyQt5.QtCore import Qt, pyqtSignal
from PyQt5.QtGui import QFont

# Import evaluation tools if available
try:
    from evaluation import (
        load_gt_map, read_messages_by_topic, occupancy_arrays_from_msgs,
        compute_coverage, compute_accessible_coverage, compute_iou, 
        compute_time_to_coverage, compute_path_length
    )
    import matplotlib
    matplotlib.use('Qt5Agg')
    from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg
    from matplotlib.figure import Figure
    import numpy as np
    import collections
    EVALUATION_AVAILABLE = True
except ImportError:
    EVALUATION_AVAILABLE = False

from gui.utils import YamlHighlighter
import yaml

class ConfigDetailsPage(QWidget):
    back_clicked = pyqtSignal()
    stop_requested = pyqtSignal()
    edit_requested = pyqtSignal()
    run_requested = pyqtSignal(str, dict)
    
    def __init__(self, parent=None):
        super().__init__(parent)
        self.config_path = ""
        self.config_data = {}
        self.layout = QVBoxLayout(self)
        self.layout.setContentsMargins(0, 0, 0, 0)
        self.setStyleSheet("background-color: #0f172a;") # Force background
        print("DEBUG: ConfigDetailsPage initialized")
        
        self.log_view = QTextEdit() # Pre-init
        self.stop_btn = QPushButton("STOP RUN") # Pre-init
        self.summary_text = QLabel("Loading...") # Pre-init
        self.results_table = ResultsTableWidget() # Pre-init
        self.run_combo = QComboBox() # Pre-init

        self.init_ui()

    def init_ui(self):
        # 1. Top Bar
        self.top_bar = QFrame()
        self.top_bar.setStyleSheet("background-color: #1e293b; border-bottom: 1px solid #334155;")
        self.top_bar.setFixedHeight(60) # Enforce height
        
        top_layout = QHBoxLayout(self.top_bar)
        top_layout.setContentsMargins(20, 10, 20, 10)
        
        # Back Button
        back_btn = QPushButton("← Dashboard")
        back_btn.setStyleSheet("background: transparent; color: #94a3b8; font-weight: 600; border: none; font-size: 14px;")
        back_btn.setCursor(Qt.PointingHandCursor)
        back_btn.clicked.connect(self.back_clicked.emit)
        top_layout.addWidget(back_btn)
        
        # Title
        self.title_label = QLabel("Configuration Details")
        self.title_label.setStyleSheet("font-size: 18px; font-weight: bold; color: #f8fafc; margin-left: 15px;")
        top_layout.addWidget(self.title_label)
        
        top_layout.addStretch()
        
        # Options Button
        self.btn_opts = QPushButton("⚙")
        self.btn_opts.setFixedSize(40, 32)
        self.btn_opts.setStyleSheet("QPushButton { background-color: #334155; color: #f8fafc; border: none; border-radius: 6px; font-weight: bold; font-size: 16px; } QPushButton:hover { background-color: #475569; } QPushButton::menu-indicator { image: none; }")
        self.btn_opts.setCursor(Qt.PointingHandCursor)
        
        # Options Menu
        from PyQt5.QtWidgets import QMenu, QAction
        self.opts_menu = QMenu(self)
        self.opts_menu.setStyleSheet("QMenu { background-color: #1e293b; color: #f8fafc; border: 1px solid #334155; } QMenu::item:selected { background-color: #3b82f6; }")
        
        self.opt_show_results = QAction("Show Results after Run", self, checkable=True)
        self.opt_show_results.setChecked(True)
        self.opt_gazebo_gui = QAction("Enable Gazebo GUI", self, checkable=True)
        self.opt_gazebo_gui.setChecked(False)
        self.opt_rviz_gui = QAction("Enable RViz", self, checkable=True)
        self.opt_rviz_gui.setChecked(False)
        
        self.opts_menu.addAction(self.opt_show_results)
        self.opts_menu.addSeparator()
        self.opts_menu.addAction(self.opt_gazebo_gui)
        self.opts_menu.addAction(self.opt_rviz_gui)
        self.btn_opts.setMenu(self.opts_menu)
        top_layout.addWidget(self.btn_opts)

        # Run Button
        self.run_btn = QPushButton("RUN BENCHMARK")
        self.run_btn.setFixedSize(140, 32)
        self.run_btn.setStyleSheet("QPushButton { background-color: #22c55e; color: #ffffff; border: none; border-radius: 6px; font-weight: bold; } QPushButton:hover { background-color: #16a34a; }")
        self.run_btn.setCursor(Qt.PointingHandCursor)
        self.run_btn.clicked.connect(self.on_run_clicked)
        top_layout.addWidget(self.run_btn)
        
        # Edit Button
        edit_btn = QPushButton("Edit Config")
        edit_btn.setFixedSize(110, 32)
        edit_btn.setStyleSheet("QPushButton { background-color: #334155; color: #f8fafc; border: none; border-radius: 6px; font-weight: bold; } QPushButton:hover { background-color: #475569; }")
        edit_btn.clicked.connect(self.edit_requested.emit)
        top_layout.addWidget(edit_btn)
        
        # Stop Button (Pre-inited)
        self.stop_btn.setFixedSize(100, 32)
        self.stop_btn.setStyleSheet("QPushButton { background-color: rgba(239, 68, 68, 0.2); color: #ef4444; border: 1px solid #ef4444; border-radius: 6px; font-weight: bold; } QPushButton:hover { background-color: rgba(239, 68, 68, 0.3); }")
        self.stop_btn.clicked.connect(self.stop_requested.emit)
        self.stop_btn.hide()
        top_layout.addWidget(self.stop_btn)
        
        # Add TopBar to Main Layout
        self.layout.addWidget(self.top_bar)
        
        # 2. Key Content (Tabs)
        self.tabs = QTabWidget()
        self.tabs.setStyleSheet("""
            QTabWidget::pane { border: none; background: transparent; }
            QTabBar::tab { background: transparent; color: #94a3b8; padding: 12px 20px; font-size: 14px; font-weight: 600; border-bottom: 2px solid transparent; }
            QTabBar::tab:selected { color: #6366f1; border-bottom: 2px solid #6366f1; }
            QTabBar::tab:hover:!selected { color: #cbd5e1; }
        """)
        
        self.overview_tab = QWidget()
        self.logs_tab = QWidget()
        self.monitor_tab = QWidget() # New tab
        self.analysis_tab = QWidget()
        
        self.tabs.addTab(self.overview_tab, "Overview")
        self.tabs.addTab(self.logs_tab, "Logs")
        self.tabs.addTab(self.monitor_tab, "Monitor") # New tab
        self.tabs.addTab(self.analysis_tab, "Analysis")
        
        # Add Tabs to Main Layout
        content_layout = QVBoxLayout()
        content_layout.setContentsMargins(40, 30, 40, 40)
        content_layout.addWidget(self.tabs)
        self.layout.addLayout(content_layout)
        
        # 3. Setup Tab Contents
        self.setup_logs_tab()
        self.setup_overview_tab()
        self.setup_monitor_tab()
        self.setup_analysis_tab()


    def on_run_clicked(self):
        opts = {
            "show_results": self.opt_show_results.isChecked(),
            "use_gazebo": self.opt_gazebo_gui.isChecked(),
            "use_rviz": self.opt_rviz_gui.isChecked()
        }
        self.run_requested.emit(self.config_path, opts)

    def setup_overview_tab(self):
        l = QVBoxLayout(self.overview_tab)
        l.setContentsMargins(0, 20, 0, 0)
        l.setSpacing(20)

        # 1. Config Summary Card
        summary_group = QGroupBox("Configuration Summary")
        summary_group.setStyleSheet("QGroupBox { border: 1px solid #334155; border-radius: 8px; margin-top: 10px; font-weight: bold; color: #f8fafc; } QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0 5px; }")
        sg_layout = QVBoxLayout(summary_group)

        # self.summary_text pre-inited

        # self.summary_text pre-inited
        self.summary_text.setWordWrap(True)
        self.summary_text.setStyleSheet("color: #cbd5e1; font-size: 13px; padding: 10px;")
        sg_layout.addWidget(self.summary_text)
        
        l.addWidget(summary_group)

        # 2. Results Table
        results_group = QGroupBox("Run Results")
        results_group.setStyleSheet("QGroupBox { border: 1px solid #334155; border-radius: 8px; margin-top: 10px; font-weight: bold; color: #f8fafc; } QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0 5px; }")
        rg_layout = QVBoxLayout(results_group)
        
        # self.results_table pre-inited
        
        # self.results_table pre-inited
        rg_layout.addWidget(self.results_table)
        self.results_table.run_deleted.connect(self.delete_run_folder)
        
        # Refresh Button for table
        refresh_btn = QPushButton("Refresh Results")
        refresh_btn.setFixedSize(120, 28)
        refresh_btn.setStyleSheet("QPushButton { background-color: #334155; color: #f8fafc; border: none; border-radius: 4px; } QPushButton:hover { background-color: #475569; }")
        refresh_btn.clicked.connect(self.scan_results) 
        rg_layout.addWidget(refresh_btn, alignment=Qt.AlignRight)
        
        l.addWidget(results_group)


    def setup_logs_tab(self):
        l = QVBoxLayout(self.logs_tab)
        l.setContentsMargins(0, 20, 0, 0)
        # self.log_view already created
        self.log_view.setReadOnly(True)
        self.log_view.setStyleSheet("font-family: Monospace; font-size: 12px; background-color: #0f172a; border: 1px solid #334155; border-radius: 8px; padding: 10px;")
        l.addWidget(self.log_view)

    def setup_monitor_tab(self):
        l = QVBoxLayout(self.monitor_tab)
        l.setContentsMargins(0, 20, 0, 0)
        l.setSpacing(20)

        # 1. Stats Row
        stats_layout = QHBoxLayout()
        self.cpu_card = self._create_stat_card("Current CPU", "0.0 %", "#6366f1")
        self.ram_card = self._create_stat_card("Current RAM", "0.0 MB", "#10b981")
        stats_layout.addWidget(self.cpu_card)
        stats_layout.addWidget(self.ram_card)
        l.addLayout(stats_layout)

        # 2. Charts
        self.max_points = 60
        self.cpu_history = collections.deque([0.0] * self.max_points, maxlen=self.max_points)
        self.ram_history = collections.deque([0.0] * self.max_points, maxlen=self.max_points)
        self.traj_x = []
        self.traj_y = []
        self.time_data = list(range(self.max_points))

        main_viz_layout = QHBoxLayout() # Split Charts and Trajectory
        
        # Left: CPU/RAM
        perf_frame = QFrame()
        perf_frame.setStyleSheet("background-color: #1e293b; border-radius: 12px; border: 1px solid #334155;")
        perf_vbox = QVBoxLayout(perf_frame)
        
        self.mon_figure = Figure(figsize=(6, 6), facecolor='#1e293b')
        self.mon_canvas = FigureCanvasQTAgg(self.mon_figure)
        perf_vbox.addWidget(self.mon_canvas)
        
        self.ax_cpu = self.mon_figure.add_subplot(211)
        self.ax_ram = self.mon_figure.add_subplot(212)
        
        for ax in [self.ax_cpu, self.ax_ram]:
            ax.set_facecolor('#1e293b')
            ax.tick_params(colors='#94a3b8', labelsize=8)
            for spine in ax.spines.values():
                spine.set_color('#334155')
        
        self.mon_cpu_line, = self.ax_cpu.plot(self.time_data, list(self.cpu_history), color='#6366f1', linewidth=2)
        self.mon_ram_line, = self.ax_ram.plot(self.time_data, list(self.ram_history), color='#10b981', linewidth=2)
        
        self.ax_cpu.set_title("CPU Usage (%)", color='#f1f5f9', fontsize=10, loc='left')
        self.ax_ram.set_title("RAM Usage (MB)", color='#f1f5f9', fontsize=10, loc='left')
        self.mon_figure.tight_layout(pad=3.0)
        
        main_viz_layout.addWidget(perf_frame, 2)

        # Right: Trajectory
        traj_frame = QFrame()
        traj_frame.setStyleSheet("background-color: #1e293b; border-radius: 12px; border: 1px solid #334155;")
        traj_vbox = QVBoxLayout(traj_frame)
        
        self.traj_figure = Figure(figsize=(6, 6), facecolor='#1e293b')
        self.traj_canvas = FigureCanvasQTAgg(self.traj_figure)
        traj_vbox.addWidget(self.traj_canvas)
        
        self.ax_traj = self.traj_figure.add_subplot(111)
        self.ax_traj.set_facecolor('#0f172a')
        self.ax_traj.tick_params(colors='#94a3b8', labelsize=8)
        self.ax_traj.grid(True, color='#334155', linestyle='--', alpha=0.5)
        self.ax_traj.set_title("Live Trajectory (Odom)", color='#f1f5f9', fontsize=10)
        
        self.traj_line, = self.ax_traj.plot([], [], color='#f43f5e', linewidth=2, label='Path')
        self.robot_dot, = self.ax_traj.plot([], [], 'o', color='#ffffff', markersize=6, label='Robot')
        
        self.traj_figure.tight_layout()
        main_viz_layout.addWidget(traj_frame, 3)

        l.addLayout(main_viz_layout)
        
        self.mon_info_lbl = QLabel("No active run.")
        self.mon_info_lbl.setStyleSheet("color: #94a3b8; font-style: italic;")
        l.addWidget(self.mon_info_lbl)

    def _create_stat_card(self, title, value, color):
        card = QFrame()
        card.setStyleSheet(f"""
            QFrame {{ background-color: #1e293b; border: 1px solid #334155; border-left: 4px solid {color}; border-radius: 8px; padding: 15px; }}
        """)
        l = QVBoxLayout(card)
        t = QLabel(title)
        t.setStyleSheet("color: #94a3b8; font-size: 12px; font-weight: bold; border: none;")
        v = QLabel(value)
        v.setStyleSheet(f"color: {color}; font-size: 24px; font-weight: bold; border: none;")
        v.setObjectName("valueLabel")
        l.addWidget(t)
        l.addWidget(v)
        return card

    def update_monitor(self, data):
        cpu = data.get('cpu', 0.0)
        ram = data.get('ram', 0.0)
        pose = data.get('pose', {})
        
        # Update Cards
        self.cpu_card.findChild(QLabel, "valueLabel").setText(f"{cpu} %")
        self.ram_card.findChild(QLabel, "valueLabel").setText(f"{ram} MB")
        
        # Update Stats Data
        self.cpu_history.append(cpu)
        self.ram_history.append(ram)
        
        # Update Perf Plots
        self.mon_cpu_line.set_ydata(list(self.cpu_history))
        self.mon_ram_line.set_ydata(list(self.ram_history))
        self.ax_cpu.relim()
        self.ax_cpu.autoscale_view()
        self.ax_ram.relim()
        self.ax_ram.autoscale_view()
        self.mon_canvas.draw()

        # Update Trajectory
        if pose:
            px, py = pose.get('x', 0.0), pose.get('y', 0.0)
            self.traj_x.append(px)
            self.traj_y.append(py)
            
            self.traj_line.set_data(self.traj_x, self.traj_y)
            self.robot_dot.set_data([px], [py])
            
            # Auto-scroll/zoom traj
            self.ax_traj.relim()
            self.ax_traj.autoscale_view()
            self.traj_canvas.draw()

    def setup_analysis_tab(self):
        l = QVBoxLayout(self.analysis_tab)
        l.setContentsMargins(0, 20, 0, 0)
        
        if not EVALUATION_AVAILABLE:
            l.addWidget(QLabel("Evaluation tools unavailable."))
            return

        # Controls
        controls = QHBoxLayout()
        controls.addWidget(QLabel("Select Run:"))

        # self.run_combo pre-inited
        # self.run_combo pre-inited
        self.run_combo.setFixedWidth(300)
        self.run_combo.setStyleSheet("""
            QComboBox { background-color: #1e293b; border: 1px solid #334155; padding: 5px 10px; border-radius: 6px; color: white; }
            QComboBox::drop-down { border: none; }
        """)
        controls.addWidget(self.run_combo)
        
        analyze_btn = QPushButton("Analyze")
        analyze_btn.setObjectName("actionButton")
        analyze_btn.clicked.connect(self.run_analysis)
        controls.addWidget(analyze_btn)
        
        controls.addStretch()
        l.addLayout(controls)
        
        # Content
        splitter = QSplitter(Qt.Vertical)
        
        # Results Text
        self.results_text = QTextEdit()
        self.results_text.setReadOnly(True)
        self.results_text.setMaximumHeight(150)
        self.results_text.setFont(QFont("Monospace", 10))
        splitter.addWidget(self.results_text)
        
        # Viz
        self.figure = Figure(figsize=(10, 6))
        self.canvas = FigureCanvasQTAgg(self.figure)
        self.ax_gt = self.figure.add_subplot(121)
        self.ax_est = self.figure.add_subplot(122)
        
        # Styling plots
        self.figure.patch.set_facecolor('#0f172a')
        self.ax_gt.set_facecolor('#0f172a')
        self.ax_est.set_facecolor('#0f172a')
        
        splitter.addWidget(self.canvas)
        l.addWidget(splitter)
        
    def load_config(self, path, data):
        print(f"DEBUG: loading config {path}")
        self.config_path = path
        self.config_data = data
        self.title_label.setText(f"Configuration: {data.get('name', 'Unknown')}")
        self.log_view.clear()
        
        # Reset Monitor
        self.cpu_history = collections.deque([0.0] * self.max_points, maxlen=self.max_points)
        self.ram_history = collections.deque([0.0] * self.max_points, maxlen=self.max_points)
        self.traj_x = []
        self.traj_y = []
        self.mon_info_lbl.setText("Ready to monitor.")
        
        # Try to background-load GT map for Trajectory plot
        self.ax_traj.clear()
        self.ax_traj.set_facecolor('#0f172a')
        self.ax_traj.grid(True, color='#334155', linestyle='--', alpha=0.5)
        self.ax_traj.set_title("Live Trajectory (Odom)", color='#f1f5f9', fontsize=10)
        self.traj_line, = self.ax_traj.plot([], [], color='#f43f5e', linewidth=2, label='Path')
        self.robot_dot, = self.ax_traj.plot([], [], 'o', color='#ffffff', markersize=6, zorder=5)

        # Optimization: Check for GT map
        gt_map_path = None
        for ds in self.config_data.get("datasets", []):
            if "ground_truth" in ds:
                gt_map_path = ds["ground_truth"].get("map_path")
                break
        
        if gt_map_path and EVALUATION_AVAILABLE:
            try:
                full_gt = (Path.cwd() / gt_map_path).resolve()
                if full_gt.exists():
                    gt_map, gt_res, gt_origin = load_gt_map(str(full_gt))
                    # Plot GT in background
                    extents = [
                        gt_origin[0], 
                        gt_origin[0] + gt_map.shape[1] * gt_res,
                        gt_origin[1],
                        gt_origin[1] + gt_map.shape[0] * gt_res
                    ]
                    # Convert map to vis (0-1 free/occ)
                    vis = np.zeros(gt_map.shape)
                    vis[gt_map == 0] = 0.8 # Free -> Light Gray
                    vis[gt_map > 50] = 0.2 # Occ -> Dark Gray
                    vis[gt_map == -1] = 0.0 # Unknown -> Black
                    
                    self.ax_traj.imshow(np.flipud(vis), extent=extents, origin='lower', cmap='gray', alpha=0.3)
            except Exception as e:
                print(f"DEBUG: Could not preview GT map: {e}")

        self.mon_canvas.draw()
        self.traj_canvas.draw()
        
        # Fill Overview Summary
        name = data.get('name', 'Unknown')
        datasets = data.get('datasets', [])
        slams = data.get('slams', [])
        slam_ids = [s.get('id') for s in slams]
        
        matrix = data.get('matrix', {})
        repeats = 1
        try:
             first_inc = matrix.get('include', [])[0]
             repeats = first_inc.get('repeats', 1)
        except:
             pass

        summary = (
            f"<b>Name:</b> {name}<br>"
            f"<b>Datasets:</b> {len(datasets)} items<br>"
            f"<b>SLAM Algorithms:</b> {', '.join(slam_ids)}<br>"
            f"<b>Repeats:</b> {repeats}<br>"
            f"<b>Output Root:</b> {data.get('output', {}).get('root_dir', 'results/runs')}"
        )
        self.summary_text.setText(summary)
        
        self.scan_results()
        
    def set_running(self, is_running):
        if is_running:
            self.stop_btn.show()
            self.run_btn.hide()
        else:
            self.stop_btn.hide()
            self.run_btn.show()

    def add_log(self, text):
        self.log_view.append(text)
        
    def set_logs(self, logs):
        self.log_view.clear()
        self.log_view.append("\n".join(logs))
        # Scroll to bottom
        sb = self.log_view.verticalScrollBar()
        sb.setValue(sb.maximum())

    def clear_logs(self):
        self.log_view.clear()
        
    def scan_results(self):
        self.run_combo.clear()
        
        output_root = self.config_data.get("output", {}).get("root_dir", "results/runs")
        root_path = Path(output_root)
        
        if not root_path.exists():
            return
            
        runs = sorted([d for d in root_path.iterdir() if d.is_dir()], reverse=True)
        
        # Get allowed IDs for filtering
        datasets = self.config_data.get('datasets', [])
        allowed_datasets = [d.get('id') for d in datasets]
        
        slams = self.config_data.get('slams', [])
        allowed_slams = [s.get('id') for s in slams]
        
        # Update Table
        self.results_table.setRowCount(0)
        
        for r in runs:
            # Parse directory name: TIMESTAMP__DATASET__SLAM__...
            parts = r.name.split("__")
            if len(parts) < 3: 
                continue
            
            date_str = parts[0]
            dataset_id = parts[1]
            slam_id = parts[2]
            
            # Filter: Show only runs belonging to this config's components
            if dataset_id not in allowed_datasets or slam_id not in allowed_slams:
                continue
                
            self.run_combo.addItem(r.name, str(r))
            
            metrics = {}
            if (r / "metrics.json").exists():
                try:
                    with open(r / "metrics.json") as f:
                        metrics = json.load(f)
                except:
                    pass
            
            val = metrics.get('ate_rmse')
            rmse = f"{val:.4f}" if val is not None else "-"
            # We could fetch path length from metrics if saved, or just leave as -
            path_len = "-"
            
            status = "Completed" if (r / "config_resolved.yaml").exists() else "Incomplete"
            if not (r / "bags" / "output").exists():
                status = "No Data"
                
            self.results_table.add_run(date_str, dataset_id, slam_id, status, rmse, path_len, str(r))
            
    def delete_run_folder(self, path):
        import shutil
        try:
            shutil.rmtree(path)
            self.scan_results()
        except Exception as e:
            from PyQt5.QtWidgets import QMessageBox
            QMessageBox.critical(self, "Error", f"Failed to delete run: {e}")

    def run_analysis(self):
        run_name = self.run_combo.currentText()
        run_path = self.run_combo.currentData()
        
        if not run_path:
            return
            
        self.results_text.setText(f"Analyzing {run_name}...")
        self.results_text.append(f"Path: {run_path}")
        
        # 1. Find Rosbag
        from pathlib import Path
        import os
        
        bag_dir = Path(run_path) / "bags" / "output"
        if not bag_dir.exists():
            self.results_text.append("❌ Status: No rosbag found (bags/output missing).")
            return
            
        # 2. Find GT Map from Config
        # We need to find which dataset was used in this run to get the GT path
        # We can try to read the resolved config from the run folder
        resolved_config_path = Path(run_path) / "config_resolved.yaml"
        gt_path = None
        
        if resolved_config_path.exists():
            import yaml
            try:
                with open(resolved_config_path, 'r') as f:
                    cfg = yaml.safe_load(f)
                    # Check dataset for ground_truth
                    ds = cfg.get("dataset", {})
                    gt_def = ds.get("ground_truth", {})
                    if gt_def:
                        gt_path = gt_def.get("map_path")
            except Exception as e:
                self.results_text.append(f"⚠️ Warning: Could not read config_resolved.yaml: {e}")
        
        # Fallback: check raw matrix config if resolved missing (less reliable)
        if not gt_path:
             # Try to find first dataset in matrix with GT
             for ds in self.config_data.get("datasets", []):
                 if "ground_truth" in ds:
                     gt_path = ds["ground_truth"].get("map_path")
                     break

        if not gt_path:
            self.results_text.append("❌ Error: No Ground Truth map defined in configuration.")
            self.results_text.append("Tip: Add 'ground_truth: {map_path: ...}' to your dataset definition.")
            return

        # Resolve GT Path (relative to project root)
        project_root = Path.cwd() # Assuming CWD is project root
        full_gt_path = (project_root / gt_path).resolve()
        
        if not full_gt_path.exists():
            self.results_text.append(f"❌ Error: GT Map file not found at: {full_gt_path}")
            return
            
        self.results_text.append(f"✅ GT Map found: {gt_path}")
        
        # Run Evaluation
        try:
            self.results_text.append("Loading data... (this may take a moment)")
            # Force UI update
            from PyQt5.QtWidgets import QApplication
            QApplication.processEvents()
            
            # 1. Load GT
            gt_map, gt_res, gt_origin = load_gt_map(str(full_gt_path))
            
            # 2. Read Bag
            self.results_text.append(f"Reading rosbag: {bag_dir}")
            
            # Auto-detect db3 file or just pass the directory
            # The reader usually expects the directory for split bags, or the db3 file
            # Let's pass the directory string which is standard for ROS2 bag reader
            bag_path_str = str(bag_dir)
            
            map_data = read_messages_by_topic(bag_path_str, ["/map"])
            map_msgs = map_data.get("/map", [])
            
            odom_data = read_messages_by_topic(bag_path_str, ["/odom"])
            odom_msgs = odom_data.get("/odom", [])
            
            if not map_msgs:
                self.results_text.append("❌ Error: No /map messages found in rosbag.")
                return

            # 3. Compute Metrics
            self.results_text.append("Aligning and computing metrics...")
            est_map = occupancy_arrays_from_msgs(map_msgs, gt_map, gt_res, gt_origin)
            _, last_map_msg = map_msgs[-1]
            
            final_cov = compute_coverage(gt_map, est_map)
            accessible_cov = compute_accessible_coverage(
                gt_map, est_map, gt_res, gt_origin,
                (last_map_msg.info.origin.position.x, last_map_msg.info.origin.position.y),
                last_map_msg.info.width, last_map_msg.info.height, last_map_msg.info.resolution
            )
            final_iou = compute_iou(gt_map, est_map)
            path_len = compute_path_length(odom_msgs, bag_path_str)
            
            # 4. Display Results
            self.results_text.append("\n=== BENCHMARK RESULTS ===")
            self.results_text.append(f"Total Map Coverage       : {final_cov*100:.2f} %")
            self.results_text.append(f"Accessible Area Coverage : {accessible_cov*100:.2f} %")
            self.results_text.append(f"Final Occupancy IoU      : {final_iou:.4f}")
            self.results_text.append(f"Total Path Length        : {path_len:.2f} m")
            
            # 5. Visualize
            self.update_maps(gt_map, est_map)
            
        except Exception as e:
            self.results_text.append(f"\n❌ Evaluation Failed: {str(e)}")
            import traceback
            traceback.print_exc()

    def update_maps(self, gt_map, est_map):
        # Helper to visualize
        def to_vis(grid):
            vis = np.zeros((*grid.shape, 4), dtype=np.uint8)
            # -1 -> transparent
            # 0 (free) -> white (255)
            # 100 (occ) -> black (0)
            
            # Background transparent
            vis[:, :, 3] = 0
            
            # Free cells -> White, Opaque
            mask_free = (grid == 0)
            vis[mask_free] = [255, 255, 255, 255]
            
            # Occupied -> Black, Opaque
            mask_occ = (grid > 50)
            vis[mask_occ] = [0, 0, 0, 255]
            
            # Unknown -> keep transparent (or gray if preferred)
            return vis

        # GT map is already flipped by the generator (np.flipud in save)
        # So we need to flip the estimated map to match for visualization
        gt_vis = to_vis(gt_map)
        est_vis = to_vis(np.flipud(est_map))
        
        self.ax_gt.clear()
        self.ax_est.clear()
        
        self.ax_gt.set_title("Ground Truth", color='white')
        self.ax_est.set_title("Estimated", color='white')
        
        self.ax_gt.imshow(gt_vis, cmap='gray')
        self.ax_est.imshow(est_vis, cmap='gray')
        
        self.ax_gt.axis('off')
        self.ax_est.axis('off')
        
        self.canvas.draw()

    def open_run_results(self, run_path):
        self.tabs.setCurrentIndex(2) # Analysis
        self.scan_results() # Refresh list
        
        # Find run in combo
        idx = self.run_combo.findData(str(run_path))
        if idx >= 0:
            self.run_combo.setCurrentIndex(idx)
            self.run_analysis()
        else:
            self.results_text.setText(f"Could not find run {run_path} in list.")

class ResultsTableWidget(QTableWidget):
    run_deleted = pyqtSignal(str) # Path

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setColumnCount(6)
        self.setHorizontalHeaderLabels(["Date", "Dataset", "SLAM", "Status", "RMSE", "Path"])
        self.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.verticalHeader().setVisible(False)
        self.setSelectionBehavior(QTableWidget.SelectRows)
        self.setContextMenuPolicy(Qt.CustomContextMenu)
        self.customContextMenuRequested.connect(self.open_menu)
        self.setStyleSheet("""
            QTableWidget { background-color: rgba(15, 23, 42, 0.6); border: 1px solid #334155; border-radius: 6px; color: #e2e8f0; gridline-color: #334155; }
            QHeaderView::section { background-color: #1e293b; color: #94a3b8; padding: 6px; border: none; font-weight: bold; }
            QTableWidget::item { padding: 5px; }
            QTableWidget::item:selected { background-color: #3b82f6; color: white; }
        """)
        
    def add_run(self, date, dataset, slam, status, rmse, path_len, full_path):
        row = self.rowCount()
        self.insertRow(row)
        
        item_date = QTableWidgetItem(date)
        item_date.setData(Qt.UserRole, full_path) # Store path
        self.setItem(row, 0, item_date)
        
        self.setItem(row, 1, QTableWidgetItem(dataset))
        self.setItem(row, 2, QTableWidgetItem(slam))
        
        status_item = QTableWidgetItem(status)
        if status == "Completed":
            status_item.setForeground(Qt.green)
        elif status == "Failed":
             status_item.setForeground(Qt.red)
        self.setItem(row, 3, status_item)
        
        rmse_item = QTableWidgetItem(rmse)
        if rmse != "-" and float(rmse) < 0.1:
             rmse_item.setForeground(Qt.green)
        self.setItem(row, 4, rmse_item)
        
        self.setItem(row, 5, QTableWidgetItem(path_len))

    def open_menu(self, position):
        row = self.indexAt(position).row()
        if row < 0: return
        
        from PyQt5.QtWidgets import QMenu, QAction, QMessageBox
        
        menu = QMenu()
        delete_action = QAction("Delete Run", self)
        menu.addAction(delete_action)
        
        action = menu.exec_(self.viewport().mapToGlobal(position))
        
        if action == delete_action:
            item = self.item(row, 0)
            path = item.data(Qt.UserRole)
            
            reply = QMessageBox.question(self, "Delete Run", 
                f"Are you sure you want to delete this run?\n{path}",
                QMessageBox.Yes | QMessageBox.No)
                
            if reply == QMessageBox.Yes:
                self.run_deleted.emit(path)

    def keyPressEvent(self, event):
        if event.key() == Qt.Key_Delete:
            row = self.currentRow()
            if row >= 0:
                item = self.item(row, 0)
                path = item.data(Qt.UserRole)
                from PyQt5.QtWidgets import QMessageBox
                reply = QMessageBox.question(self, "Delete Run", 
                    f"Are you sure you want to delete this run?\n{path}",
                    QMessageBox.Yes | QMessageBox.No)
                if reply == QMessageBox.Yes:
                    self.run_deleted.emit(path)
        super().keyPressEvent(event)
