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

class SimulatorManagementPage(QWidget):
    """Page for managing simulators (Gazebo, O3DE)"""
    
    def __init__(self, parent=None):
        super().__init__(parent)
        self.layout = QVBoxLayout(self)
        self.layout.setContentsMargins(20, 20, 20, 20)
        
        # Store widget references instead of using findChild()
        self.widgets = {}  # {sim_name: {'status': QLabel, 'details': QTextEdit, 'install_btn': QPushButton}}
        
        self.init_ui()
    
    def init_ui(self):
        # Header
        header = QLabel("Simulator Management")
        header.setObjectName("headerLabel")
        self.layout.addWidget(header)
        
        # Import simulator manager
        try:
            from tools.simulator_manager import SimulatorManager
            self.sim_mgr = SimulatorManager()
        except ImportError as e:
            error_label = QLabel(f"❌ Error loading SimulatorManager: {e}")
            error_label.setStyleSheet("color: #ef4444; padding: 20px;")
            self.layout.addWidget(error_label)
            return
        
        # Simulator Cards
        for sim_name in ['gazebo', 'o3de']:
            card = self._create_simulator_card(sim_name)
            self.layout.addWidget(card)
        
        self.layout.addStretch()
        
        # Refresh status AFTER widgets are in layout (use QTimer to ensure Qt event loop is ready)
        from PyQt5.QtCore import QTimer
        QTimer.singleShot(100, self.refresh_all_statuses)
    
    def refresh_all_statuses(self):
        """Refresh all simulator statuses"""
        for sim_name in self.widgets.keys():
            self.refresh_simulator_status(sim_name)
    
    def _create_simulator_card(self, sim_name: str) -> QFrame:
        """Create a card for a simulator"""
        card = QFrame()
        card.setProperty("class", "card")
        card_layout = QVBoxLayout(card)
        
        # Title
        title = QLabel(sim_name.upper())
        title.setStyleSheet("font-size: 18px; font-weight: bold; color: #f8fafc;")
        card_layout.addWidget(title)
        
        # Status label
        status_label = QLabel("Checking...")
        status_label.setStyleSheet("color: #94a3b8; font-size: 14px; margin: 10px 0;")
        card_layout.addWidget(status_label)
        
        # Details
        details_text = QTextEdit()
        details_text.setReadOnly(True)
        details_text.setMaximumHeight(120)
        details_text.setStyleSheet("background-color: #1e293b; color: #cbd5e1; border: 1px solid #334155; border-radius: 4px; font-family: monospace; font-size: 12px;")
        card_layout.addWidget(details_text)
        
        # Action buttons
        btn_layout = QHBoxLayout()
        
        install_btn = QPushButton(f"Install {sim_name.upper()}")
        install_btn.setStyleSheet("""
            QPushButton { background-color: #6366f1; color: white; border: none; border-radius: 6px; padding: 8px 16px; font-weight: bold; }
            QPushButton:hover { background-color: #4f46e5; }
            QPushButton:disabled { background-color: #334155; color: #64748b; }
        """)
        install_btn.clicked.connect(lambda: self.install_simulator(sim_name))
        btn_layout.addWidget(install_btn)
        
        refresh_btn = QPushButton("Refresh")
        refresh_btn.setStyleSheet("""
            QPushButton { background-color: #334155; color: #f8fafc; border: none; border-radius: 6px; padding: 8px 16px; }
            QPushButton:hover { background-color: #475569; }
        """)
        refresh_btn.clicked.connect(lambda: self.refresh_simulator_status(sim_name))
        btn_layout.addWidget(refresh_btn)
        
        btn_layout.addStretch()
        card_layout.addLayout(btn_layout)
        
        # Store widget references
        self.widgets[sim_name] = {
            'status': status_label,
            'details': details_text,
            'install_btn': install_btn
        }
        
        return card
    
    def refresh_simulator_status(self, sim_name: str):
        """Refresh simulator status"""
        try:
            sim = self.sim_mgr.get_simulator(sim_name)
            if not sim:
                return
            
            # Get widgets
            if sim_name not in self.widgets:
                return
            
            status_label = self.widgets[sim_name]['status']
            details_text = self.widgets[sim_name]['details']
            install_btn = self.widgets[sim_name]['install_btn']
            
            # Check installation
            installed = sim.is_installed()
            version = sim.get_version()
            deps = sim.verify_dependencies()
            size_mb = sim.get_install_size_mb()
            
            # Update status label
            if installed:
                status_label.setText(f"✅ Installed - Version: {version or 'Unknown'}")
                status_label.setStyleSheet("color: #10b981; font-size: 14px; margin: 10px 0;")
            else:
                status_label.setText(f"❌ Not Installed (Size: ~{size_mb} MB)")
                status_label.setStyleSheet("color: #ef4444; font-size: 14px; margin: 10px 0;")
            
            # Update details
            details = f"Installation Directory: {sim.install_dir}\n\n"
            details += "Dependencies:\n"
            for dep, available in deps.items():
                status = "✅" if available else "❌"
                details += f"  {status} {dep}\n"
            details_text.setText(details)
            
            # Update button
            if installed:
                install_btn.setEnabled(False)
                install_btn.setText(f"{sim_name.upper()} Already Installed")
            else:
                install_btn.setEnabled(True)
                install_btn.setText(f"Install {sim_name.upper()}")
        
        except Exception as e:
            # Show error in details instead of crashing
            if sim_name in self.widgets:
                self.widgets[sim_name]['details'].setText(f"Error checking status: {e}")
                self.widgets[sim_name]['status'].setText("⚠️ Error")
                self.widgets[sim_name]['status'].setStyleSheet("color: #f59e0b; font-size: 14px;")

    
    def install_simulator(self, sim_name: str):
        """Trigger simulator installation"""
        from PyQt5.QtCore import QThread, pyqtSignal
        
        # Confirm installation
        sim = self.sim_mgr.get_simulator(sim_name)
        size_mb = sim.get_install_size_mb()
        
        reply = QMessageBox.question(
            self, 
            f"Install {sim_name.upper()}", 
            f"This will download and build {sim_name.upper()} (~{size_mb} MB).\n"
            f"This may take 30-60 minutes depending on your system.\n\n"
            f"Continue?",
            QMessageBox.Yes | QMessageBox.No
        )
        
        if reply != QMessageBox.Yes:
            return
        
        # Create installation worker thread
        class InstallWorker(QThread):
            progress = pyqtSignal(str, int)
            finished = pyqtSignal(bool)
            
            def __init__(self, sim):
                super().__init__()
                self.sim = sim
            
            def run(self):
                success = self.sim.install(progress_callback=self.emit_progress)
                self.finished.emit(success)
            
            def emit_progress(self, message, percent):
                self.progress.emit(message, percent)
        
        # Disable button during installation
        install_btn = self.widgets[sim_name]['install_btn']
        install_btn.setEnabled(False)
        install_btn.setText("Installing...")
        
        # Create enhanced progress dialog
        from PyQt5.QtWidgets import QProgressDialog, QLabel
        progress_dialog = QProgressDialog(self)
        progress_dialog.setWindowTitle(f"Installing {sim_name.upper()}")
        progress_dialog.setWindowModality(Qt.WindowModal)
        progress_dialog.setAutoClose(False)  # Don't auto-close
        progress_dialog.setMinimumDuration(0)
        progress_dialog.setMinimumWidth(500)
        progress_dialog.setCancelButton(None)  # No cancel during install
        progress_dialog.setRange(0, 100)
        
        # Custom label for detailed info
        info_label = QLabel("Starting installation...")
        info_label.setStyleSheet("color: #1e293b; padding: 10px; font-size: 13px;")
        info_label.setWordWrap(True)
        progress_dialog.setLabel(info_label)
        
        # Start installation
        self.install_worker = InstallWorker(sim)  # Keep reference to avoid GC
        worker = self.install_worker
        
        import time
        start_time = time.time()
        last_percent = 0
        
        def format_time(seconds):
            """Format seconds into human-readable time"""
            if seconds < 60:
                return f"{seconds}s"
            elif seconds < 3600:
                mins = seconds // 60
                secs = seconds % 60
                return f"{mins}m {secs}s"
            else:
                hours = seconds // 3600
                mins = (seconds % 3600) // 60
                return f"{hours}h {mins}m"
        
        def update_progress(message, percent):
            nonlocal last_percent
            
            # Calculate elapsed and remaining time
            elapsed = time.time() - start_time
            
            if percent > 5 and percent != last_percent:  # Avoid division by zero
                # Estimate total time based on current progress
                estimated_total = (elapsed / percent) * 100
                remaining = estimated_total - elapsed
                
                # Format time
                elapsed_str = format_time(int(elapsed))
                remaining_str = format_time(int(remaining))
                
                # Update label with detailed info
                detailed_msg = f"{message}\n\n"
                detailed_msg += f"Progress: {percent}%\n"
                detailed_msg += f"⏱️  Elapsed: {elapsed_str}\n"
                detailed_msg += f"⏳ Estimated remaining: {remaining_str}"
                
                info_label.setText(detailed_msg)
            else:
                info_label.setText(f"{message}\n\nProgress: {percent}%")
            
            progress_dialog.setValue(percent)
            last_percent = percent
        
        def on_finished(success):
            progress_dialog.close()
            if success:
                elapsed_total = time.time() - start_time
                QMessageBox.information(
                    self, 
                    "Success", 
                    f"{sim_name.upper()} installed successfully!\n\n"
                    f"⏱️ Total time: {format_time(int(elapsed_total))}"
                )
            else:
                QMessageBox.warning(
                    self, 
                    "Installation Failed", 
                    f"Failed to install {sim_name.upper()}.\n"
                    f"Check the console for details."
                )
            self.refresh_simulator_status(sim_name)
        
        worker.progress.connect(update_progress)
        worker.finished.connect(on_finished)
        worker.start()


class CleanupPage(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.layout = QVBoxLayout(self)
        self.layout.setContentsMargins(20, 20, 20, 20)
        
        header = QLabel("Deep Clean / Process Kill")
        header.setStyleSheet("font-size: 18px; font-weight: bold; color: #f8fafc;")
        self.layout.addWidget(header)
        
        desc = QLabel("Use this if simulations are getting stuck or failing to start.\nIt forcefully kills known ROS 2, Gazebo, and O3DE processes.")
        desc.setStyleSheet("color: #94a3b8; font-size: 14px; margin-bottom: 20px;")
        self.layout.addWidget(desc)
        
        # Action Button
        kill_btn = QPushButton("🔥 Kill All Zombie Processes")
        kill_btn.setStyleSheet("""
            QPushButton { background-color: #ef4444; color: white; padding: 15px; border-radius: 8px; font-size: 16px; font-weight: bold; }
            QPushButton:hover { background-color: #dc2626; }
        """)
        kill_btn.clicked.connect(self.kill_all)
        self.layout.addWidget(kill_btn)
        
        self.log_area = QTextEdit()
        self.log_area.setReadOnly(True)
        self.log_area.setStyleSheet("background-color: #1e293b; color: #cbd5e1; font-family: monospace; margin-top: 20px;")
        self.layout.addWidget(self.log_area)
        
    def kill_all(self):
        self.log_area.clear()
        import subprocess
        
        targets = [
            "gzserver", "gzclient", "ruby", # Gazebo
            "Editor", "GameLauncher", "AssetProcessor", # O3DE
            "ros2", "_ros2_daemon", # ROS 2
            "nav2_manager", "component_container", "component_container_isolated",
            "robot_state_publisher", "rviz2",
            "explore", "map_server", "amcl"
        ]
        
        self.log_area.append("Killing processes...")
        
        count = 0
        for t in targets:
            try:
                # pkill -f matches command line similar to regex
                # We use -9 (SIGKILL) to be sure
                res = subprocess.run(["pkill", "-9", "-f", t], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
                if res.returncode == 0:
                    self.log_area.append(f"✅ Killed: {t}")
                    count += 1
                else:
                    self.log_area.append(f"⚪ Not found: {t}")
            except Exception as e:
                self.log_area.append(f"❌ Error killing {t}: {e}")
        
        self.log_area.append(f"\nDone. {count} process group(s) terminated.")
        self.log_area.append("You can now try running your configuration again.")

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
        self.sim_page = SimulatorManagementPage()
        self.cleanup_page = CleanupPage()
        
        self.tabs.addTab(self.gt_page, "GT Generator")
        self.tabs.addTab(self.manual_page, "Manual Analysis")
        self.tabs.addTab(self.sim_page, "Simulators")
        self.tabs.addTab(self.cleanup_page, "Cleanup")
        
        layout.addWidget(self.tabs)
