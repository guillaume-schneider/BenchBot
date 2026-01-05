import sys
import os
from pathlib import Path

# Add project root to sys.path
PROJECT_ROOT = Path(__file__).parent.parent
sys.path.append(str(PROJECT_ROOT))

from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, 
    QPushButton, QStackedWidget, QLabel, QFrame, QButtonGroup
)
from PyQt5.QtCore import Qt, QSize
from PyQt5.QtGui import QIcon, QFont

from gui.utils import STYLE_SHEET
from gui.worker import RunWorker
from gui.pages.dashboard import DashboardPage
from gui.pages.details import ConfigDetailsPage
from gui.pages.tools import ToolsPage 
from gui.pages.editor import ConfigEditorPage
from gui.pages.benchmark import BenchmarkPage # New Import

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("SLAM Bench Orchestrator")
        self.resize(1400, 900)
        self.setStyleSheet(STYLE_SHEET)
        
        self.active_runs = {} # {config_path: worker}
        self.running_config = None # Last focused run
        self.log_buffers = {} # {config_path: [logs]} 
        
        self.init_ui()

    def init_ui(self):
        # ... (Same as before)
        # Main Layout
        main_widget = QWidget()
        main_widget.setObjectName("mainScreen")
        self.setCentralWidget(main_widget)
        main_layout = QHBoxLayout(main_widget)
        main_layout.setContentsMargins(0, 0, 0, 0)
        main_layout.setSpacing(0)

        # Sidebar
        self.sidebar = QWidget()
        self.sidebar.setObjectName("sidebar")
        self.sidebar.setFixedWidth(260)
        sidebar_layout = QVBoxLayout(self.sidebar)
        sidebar_layout.setContentsMargins(0, 30, 0, 30)
        
        # Logo / Title
        title = QLabel("SLAM Bench")
        title.setStyleSheet("font-size: 24px; font-weight: 800; color: #f8fafc; padding-left: 30px; margin-bottom: 30px;")
        sidebar_layout.addWidget(title)
        
        # Nav Buttons
        self.nav_group = QButtonGroup(self)
        self.nav_group.setExclusive(True)
        
        self.btn_dash = self.create_nav_button("Dashboard", 0)
        self.btn_bench = self.create_nav_button("Benchmark", 4) # New Index 4
        self.btn_tools = self.create_nav_button("Tools", 2)
        
        sidebar_layout.addWidget(self.btn_dash)
        sidebar_layout.addWidget(self.btn_bench)
        sidebar_layout.addWidget(self.btn_tools)
        
        sidebar_layout.addStretch()
        
        # Version
        ver = QLabel("v2.2.0")
        ver.setStyleSheet("color: #475569; padding-left: 30px; font-weight: 600;")
        sidebar_layout.addWidget(ver)
        
        main_layout.addWidget(self.sidebar)

        # Content Area (Stacked)
        self.stack = QStackedWidget()
        main_layout.addWidget(self.stack)
        
        # Pages
        self.page_dashboard = DashboardPage()
        self.page_details = ConfigDetailsPage()
        self.page_tools = ToolsPage()
        self.page_editor = ConfigEditorPage()
        self.page_benchmark = BenchmarkPage() # Initialize
        
        self.stack.addWidget(self.page_dashboard) # 0
        self.stack.addWidget(self.page_details)   # 1
        self.stack.addWidget(self.page_tools)     # 2
        self.stack.addWidget(self.page_editor)    # 3
        self.stack.addWidget(self.page_benchmark) # 4
        
        # Connect Signals
        
        # Dashboard -> Details
        self.page_dashboard.config_selected.connect(self.show_config_details)
        # Dashboard -> Run
        self.page_dashboard.run_requested.connect(self.run_config_from_dashboard)
        # Dashboard -> Stop
        self.page_dashboard.stop_requested.connect(self.stop_worker)
        # Dashboard -> Edit
        self.page_dashboard.edit_requested.connect(self.show_config_editor)
        
        # Details -> Back
        self.page_details.back_clicked.connect(lambda: self.switch_page(0))
        # Details -> Stop
        self.page_details.stop_requested.connect(lambda: self.stop_worker(self.page_details.config_path))
        # Details -> Run
        self.page_details.run_requested.connect(self.start_worker)
        # Details -> Edit
        self.page_details.edit_requested.connect(lambda: self.show_config_editor(self.page_details.config_path))
        
        # Editor -> Back
        self.page_editor.back_clicked.connect(lambda: self.switch_page(0))
        # Editor -> Save
        self.page_editor.save_clicked.connect(self.on_config_saved)

        self.page_dashboard.refresh_configs()
        self.btn_dash.setChecked(True)

    def create_nav_button(self, text, index):
        btn = QPushButton(text)
        btn.setObjectName("navButton")
        btn.setCheckable(True)
        btn.clicked.connect(lambda: self.switch_page(index))
        self.nav_group.addButton(btn)
        return btn

    def stop_worker(self, path):
        if path in self.active_runs:
            worker = self.active_runs[path]
            if worker.isRunning():
                 worker.cancel()
                 self.handle_log("STOP requested by user...", path)

    def switch_page(self, index):
        if index == 0:
            self.btn_dash.setChecked(True)
            # Sync dashboard state when showing it
            self.page_dashboard.refresh_configs() # We need to update status for all
            # Provide running status for all active runs
            for path, worker in self.active_runs.items():
                if path in self.page_dashboard.cards:
                    self.page_dashboard.cards[path].set_running(True)
        elif index == 2:
            self.btn_tools.setChecked(True)
        elif index == 4:
            self.btn_bench.setChecked(True)
            self.page_benchmark.refresh_data() # Auto refresh
            
        self.stack.setCurrentIndex(index)

    def show_config_details(self, path, data):
        self.page_details.load_config(path, data)
        
        # Sync logs if this is a running config
        if path in self.active_runs:
            self.page_details.set_logs(self.log_buffers.get(path, []))
            self.page_details.set_running(True)
        else:
            self.page_details.set_running(False)
            self.page_details.clear_logs() # Or show old logs if we persisted them?
        
        self.switch_page(1) # Go to details

    def show_config_editor(self, path):
        self.page_editor.load_config(path)
        self.switch_page(3) # Editor

    def on_config_saved(self, path, data):
        self.page_dashboard.refresh_configs()
        self.switch_page(0) # Back to dashboard

    def run_config_from_dashboard(self, path, data):
        self.start_worker(path)

    def start_worker(self, config_path, options=None):
        if config_path in self.active_runs:
             from PyQt5.QtWidgets import QMessageBox
             QMessageBox.warning(self, "Busy", "This configuration is already running.")
             return

        if self.active_runs:
            from PyQt5.QtWidgets import QMessageBox
            reply = QMessageBox.question(self, "Concurrent Run", 
                f"There are {len(self.active_runs)} runs active.\nStart another one?",
                QMessageBox.Yes | QMessageBox.No)
            if reply == QMessageBox.No:
                return

        worker = RunWorker([config_path], use_gui=True, options=options)
        self.active_runs[config_path] = worker
        self.log_buffers[config_path] = []
        self.running_config = config_path # Set focus to this one
        
        # Connect signals with captured config_path
        # We use default arg v=config_path to capture value at loop time (though here it's function scope)
        worker.log_signal.connect(lambda msg, p=config_path: self.handle_log(msg, p))
        worker.progress_signal.connect(lambda c, t, rid, p=config_path: self.handle_progress(c, t, rid, p))
        worker.finished_signal.connect(lambda p=config_path: self.handle_finished(p))
        worker.config_started.connect(lambda p_str, p=config_path: self.handle_config_started(p_str, p))
        worker.result_ready.connect(lambda r_p, p=config_path: self.handle_result_ready(r_p, p))
        
        # Update UI state
        self.update_ui_state(config_path, True)
        worker.start()
        
        # If we are in Details page of this config, clear logs
        if self.page_details.config_path == config_path:
            self.page_details.clear_logs()

    def handle_result_ready(self, run_path, config_path):
        print(f"DEBUG: Result ready for {run_path} (Config: {config_path})")
        
        # Refresh the details page results list ONLY if we are looking at it?
        # Or always refresh. Safe to call.
        if self.page_details.config_path == config_path:
            self.page_details.scan_results()
        
        # Check options
        worker = self.active_runs.get(config_path)
        if worker:
             opts = worker.options or {}
             if opts.get("show_results", True):
                 print("DEBUG: Opening Result Window...")
                 try:
                     from gui.results_window import ResultWindow
                     # Keep reference? If multiple windows, we need a list or dict
                     if not hasattr(self, "result_windows"):
                         self.result_windows = []
                     
                     res_win = ResultWindow(run_path, self)
                     res_win.show()
                     self.result_windows.append(res_win) # prevent GC
                 except Exception as e:
                     print(f"ERROR Opening Result Window: {e}")
                     import traceback
                     traceback.print_exc()

    def handle_log(self, msg, config_path):
        if config_path not in self.log_buffers:
            self.log_buffers[config_path] = []
        self.log_buffers[config_path].append(msg)
        
        # Forward logs to Details page if it matches running config
        if self.page_details.config_path == config_path:
            self.page_details.add_log(msg)

    def handle_progress(self, current, total, run_id, config_path):
        # Update Dashboard Card
        if config_path in self.page_dashboard.cards:
            self.page_dashboard.cards[config_path].update_progress(current, total)

    def handle_config_started(self, path, config_path):
        pass

    def handle_finished(self, config_path):
        self.update_ui_state(config_path, False)
        if config_path in self.active_runs:
            del self.active_runs[config_path]
        
        if self.running_config == config_path:
            self.running_config = None

    def update_ui_state(self, path, is_running):
        # Update Dashboard Card
        if path in self.page_dashboard.cards:
            self.page_dashboard.cards[path].set_running(is_running)
            
        # Update Details Page if showing this config
        if self.page_details.config_path == path:
            self.page_details.set_running(is_running)


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())
