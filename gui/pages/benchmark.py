from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QPushButton, QLabel, 
    QTableWidget, QTableWidgetItem, QHeaderView, QAbstractItemView,
    QMessageBox, QFileDialog, QProgressDialog, QLineEdit, QComboBox, QFrame
)
from PyQt5.QtCore import Qt, QThread, pyqtSignal
from PyQt5.QtGui import QColor, QBrush
import os
import json
import yaml
from pathlib import Path

class ReportThread(QThread):
    finished = pyqtSignal(bool, str) # success, message
    
    def __init__(self, runs, path):
        super().__init__()
        self.runs = runs
        self.path = path
        
    def run(self):
        try:
            from tools.report_generator import SLAMReportGenerator
            generator = SLAMReportGenerator(self.runs, self.path)
            generator.generate()
            self.finished.emit(True, self.path)
        except Exception as e:
            import traceback
            self.finished.emit(False, f"{str(e)}\n\n{traceback.format_exc()}")

class BenchmarkPage(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setObjectName("benchmarkPage")
        self.latest_runs = []
        self.init_ui()
        
    def init_ui(self):
        layout = QVBoxLayout(self)
        layout.setContentsMargins(30, 30, 30, 30)
        layout.setSpacing(15)
        
        # Header
        header_layout = QHBoxLayout()
        title = QLabel("Global Benchmark")
        title.setStyleSheet("font-size: 24px; font-weight: bold; color: #f8fafc;")
        header_layout.addWidget(title)
        
        header_layout.addStretch()
        
        self.btn_export = QPushButton("Export PDF Report")
        self.btn_export.setObjectName("actionButton")
        self.btn_export.setStyleSheet("background-color: #6366f1; font-weight: bold;")
        self.btn_export.setFixedWidth(200)
        self.btn_export.clicked.connect(self.export_pdf_report)
        header_layout.addWidget(self.btn_export)

        self.btn_refresh = QPushButton("Refresh Data")
        self.btn_refresh.setObjectName("actionButton")
        self.btn_refresh.setFixedWidth(150)
        self.btn_refresh.clicked.connect(self.refresh_data)
        header_layout.addWidget(self.btn_refresh)
        
        layout.addLayout(header_layout)

        # --- Filter Bar ---
        filter_bar = QFrame()
        filter_bar.setStyleSheet("""
            QFrame {
                background-color: #1e293b;
                border-radius: 8px;
                padding: 10px;
            }
            QLabel { color: #94a3b8; font-weight: bold; font-size: 12px; }
            QLineEdit, QComboBox {
                background-color: #0f172a;
                color: #e2e8f0;
                border: 1px solid #334155;
                padding: 5px 10px;
                border-radius: 4px;
            }
        """)
        filter_layout = QHBoxLayout(filter_bar)
        
        # Search Run ID
        filter_layout.addWidget(QLabel("SEARCH:"))
        self.search_id = QLineEdit()
        self.search_id.setPlaceholderText("Filter by ID...")
        self.search_id.textChanged.connect(self.apply_filters)
        filter_layout.addWidget(self.search_id)
        
        # SLAM Filter
        filter_layout.addSpacing(20)
        filter_layout.addWidget(QLabel("SLAM:"))
        self.combo_slam = QComboBox()
        self.combo_slam.addItem("All")
        self.combo_slam.currentTextChanged.connect(self.apply_filters)
        filter_layout.addWidget(self.combo_slam)
        
        # Dataset Filter
        filter_layout.addSpacing(20)
        filter_layout.addWidget(QLabel("DATASET:"))
        self.combo_dataset = QComboBox()
        self.combo_dataset.addItem("All")
        self.combo_dataset.currentTextChanged.connect(self.apply_filters)
        filter_layout.addWidget(self.combo_dataset)
        
        filter_layout.addStretch()
        
        self.btn_clear = QPushButton("Reset")
        self.btn_clear.setStyleSheet("background: transparent; color: #6366f1; border: 1px solid #6366f1; padding: 4px 10px;")
        self.btn_clear.clicked.connect(self.reset_filters)
        filter_layout.addWidget(self.btn_clear)
        
        layout.addWidget(filter_bar)
        
        # Table
        self.table = QTableWidget()
        self.table.setColumnCount(10)
        self.table.setHorizontalHeaderLabels([
            "Run ID", "SLAM", "Dataset", "Duration", 
            "ATE (m)", "Coverage (%)", "IoU", "Path (m)",
            "CPU (%)", "RAM (MB)"
        ])
        self.table.horizontalHeader().setSectionResizeMode(QHeaderView.Interactive)
        self.table.horizontalHeader().setCascadingSectionResizes(True)
        self.table.horizontalHeader().setStretchLastSection(False)
        self.table.verticalHeader().setVisible(False)
        self.table.setEditTriggers(QAbstractItemView.NoEditTriggers)
        self.table.setSelectionBehavior(QAbstractItemView.SelectRows)
        self.table.setAlternatingRowColors(True)
        
        # Style
        self.table.setStyleSheet("""
            QTableWidget {
                background-color: #1e293b;
                alternate-background-color: #0f172a;
                color: #e2e8f0;
                gridline-color: #334155;
                font-size: 13px;
                border: none;
                border-radius: 8px;
            }
            QHeaderView::section {
                background-color: #0f172a;
                color: #94a3b8;
                padding: 12px;
                border: none;
                font-weight: bold;
                text-transform: uppercase;
            }
            QTableWidget::item {
                padding: 8px;
            }
            QTableWidget::item:selected {
                background-color: #3b82f6;
                color: white;
            }
        """)
        
        self.table.itemDoubleClicked.connect(self.open_result_window)
        layout.addWidget(self.table)

    def reset_filters(self):
        self.search_id.clear()
        self.combo_slam.setCurrentIndex(0)
        self.combo_dataset.setCurrentIndex(0)
        self.apply_filters()

    def apply_filters(self):
        search_text = self.search_id.text().lower()
        slam_filter = self.combo_slam.currentText()
        dataset_filter = self.combo_dataset.currentText()
        
        filtered_runs = []
        for run in self.latest_runs:
            match_search = not search_text or search_text in run['id'].lower()
            match_slam = slam_filter == "All" or run['slam'] == slam_filter
            match_dataset = dataset_filter == "All" or run['dataset'] == dataset_filter
            
            if match_search and match_slam and match_dataset:
                filtered_runs.append(run)
        
        self.populate_table(filtered_runs)

    def export_pdf_report(self):
        if not self.latest_runs:
            QMessageBox.warning(self, "No Data", "Please refresh data first.")
            return

        path, _ = QFileDialog.getSaveFileName(self, "Save Comparison Report", "slam_comparison_report.pdf", "PDF Files (*.pdf)")
        
        if path:
            self.progress = QProgressDialog("Generating PDF Report...\nThis may take a moment.", None, 0, 0, self)
            self.progress.setWindowTitle("Please Wait")
            self.progress.setWindowModality(Qt.WindowModal)
            self.progress.setMinimumDuration(0)
            self.progress.show()

            self.report_thread = ReportThread(self.latest_runs, path)
            self.report_thread.finished.connect(self.on_report_finished)
            self.report_thread.start()

    def on_report_finished(self, success, result):
        if hasattr(self, "progress"):
            self.progress.close()
        if success:
            QMessageBox.information(self, "Success", f"Report exported to:\n{result}")
        else:
            QMessageBox.critical(self, "Error", f"Failed to generate report:\n{result}")

    def open_result_window(self, item):
        row = item.row()
        run_id = self.table.item(row, 0).text()
        root = Path.cwd()
        run_dir = root / "results" / "runs" / run_id
        if run_dir.exists():
            from gui.results_window import ResultWindow
            if not hasattr(self, "result_windows"):
                self.result_windows = []
            try:
                self.result_windows = [w for w in self.result_windows if w.isVisible()]
                win = ResultWindow(run_dir, self)
                win.show()
                self.result_windows.append(win)
                self.refresh_data()
            except Exception as e:
                print(f"Error opening results: {e}")

    def refresh_data(self):
        root = Path.cwd()
        runs_dir = root / "results" / "runs"
        if not runs_dir.exists(): return
        
        runs = []
        slams = set()
        datasets = set()
        
        for run_path in runs_dir.iterdir():
            if run_path.is_dir():
                run_data = self.parse_run(run_path)
                if run_data:
                    runs.append(run_data)
                    slams.add(run_data['slam'])
                    datasets.add(run_data['dataset'])
        
        runs.sort(key=lambda x: x['id'], reverse=True)
        self.latest_runs = runs
        
        # Update combo boxes while preserving selection if possible
        cur_slam = self.combo_slam.currentText()
        cur_ds = self.combo_dataset.currentText()
        
        self.combo_slam.blockSignals(True)
        self.combo_dataset.blockSignals(True)
        
        self.combo_slam.clear()
        self.combo_slam.addItem("All")
        self.combo_slam.addItems(sorted(list(slams)))
        
        self.combo_dataset.clear()
        self.combo_dataset.addItem("All")
        self.combo_dataset.addItems(sorted(list(datasets)))
        
        # Restore selections
        idx_slam = self.combo_slam.findText(cur_slam)
        if idx_slam >= 0: self.combo_slam.setCurrentIndex(idx_slam)
        
        idx_ds = self.combo_dataset.findText(cur_ds)
        if idx_ds >= 0: self.combo_dataset.setCurrentIndex(idx_ds)
        
        self.combo_slam.blockSignals(False)
        self.combo_dataset.blockSignals(False)
        
        self.apply_filters()

    def parse_run(self, path: Path):
        data = {
            "id": path.name, "slam": "N/A", "dataset": "N/A", "duration": None,
            "ate": None, "coverage": None, "iou": None, "path": None, "cpu": None, "ram": None
        }
        parts = path.name.split("__")
        if len(parts) >= 2: data["dataset"] = parts[1]
        if len(parts) >= 3: data["slam"] = parts[2]
        metrics_path = path / "metrics.json"
        if metrics_path.exists():
            try:
                with open(metrics_path, 'r') as f:
                    m = json.load(f)
                    data["ate"] = m.get("ate_rmse")
                    data["coverage"] = m.get("coverage_percent")
                    data["iou"] = m.get("iou")
                    data["path"] = m.get("path_length_m")
                    data["duration"] = m.get("duration_s")
                    data["cpu"] = m.get("max_cpu_percent")
                    data["ram"] = m.get("max_ram_mb")
            except: pass
        return data

    def populate_table(self, runs):
        self.table.setRowCount(0)
        self.table.setRowCount(len(runs))
        for r, run in enumerate(runs):
            self.table.setItem(r, 0, QTableWidgetItem(run["id"]))
            self.table.setItem(r, 1, QTableWidgetItem(run["slam"]))
            self.table.setItem(r, 2, QTableWidgetItem(run["dataset"]))
            dur_val = run.get("duration")
            item_dur = QTableWidgetItem(f"{dur_val:.1f} s" if isinstance(dur_val, (int, float)) else "-")
            if dur_val is not None: item_dur.setTextAlignment(Qt.AlignRight | Qt.AlignVCenter)
            self.table.setItem(r, 3, item_dur)
            ate_val = run["ate"]
            item_ate = QTableWidgetItem(f"{ate_val:.3f}" if ate_val is not None else "-")
            if ate_val is not None:
                item_ate.setTextAlignment(Qt.AlignRight | Qt.AlignVCenter)
                if ate_val < 0.1: item_ate.setForeground(QColor("#4ade80"))
                elif ate_val > 1.0: item_ate.setForeground(QColor("#f87171"))
            self.table.setItem(r, 4, item_ate)
            cov_val = run["coverage"]
            item_cov = QTableWidgetItem(f"{cov_val:.1f}%" if cov_val is not None else "-")
            if cov_val is not None:
                item_cov.setTextAlignment(Qt.AlignRight | Qt.AlignVCenter)
                if cov_val > 95: item_cov.setForeground(QColor("#4ade80"))
            self.table.setItem(r, 5, item_cov)
            iou_val = run["iou"]
            item_iou = QTableWidgetItem(f"{iou_val:.3f}" if iou_val is not None else "-")
            if iou_val is not None:
                item_iou.setTextAlignment(Qt.AlignRight | Qt.AlignVCenter)
                if iou_val > 0.8: item_iou.setForeground(QColor("#4ade80"))
            self.table.setItem(r, 6, item_iou)
            path_val = run["path"]
            item_path = QTableWidgetItem(f"{path_val:.1f}" if path_val is not None else "-")
            if path_val is not None: item_path.setTextAlignment(Qt.AlignRight | Qt.AlignVCenter)
            self.table.setItem(r, 7, item_path)
            cpu_val = run["cpu"]
            item_cpu = QTableWidgetItem(f"{cpu_val:.1f}%" if cpu_val is not None else "-")
            if cpu_val is not None:
                item_cpu.setTextAlignment(Qt.AlignRight | Qt.AlignVCenter)
                if cpu_val > 150: item_cpu.setForeground(QColor("#f87171"))
            self.table.setItem(r, 8, item_cpu)
            ram_val = run["ram"]
            item_ram = QTableWidgetItem(f"{ram_val:.0f}" if ram_val is not None else "-")
            if ram_val is not None:
                item_ram.setTextAlignment(Qt.AlignRight | Qt.AlignVCenter)
                if ram_val > 2000: item_ram.setForeground(QColor("#f87171"))
            self.table.setItem(r, 9, item_ram)
        self.table.resizeColumnsToContents()
