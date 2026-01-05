from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QPushButton, QLabel, 
    QTableWidget, QTableWidgetItem, QHeaderView, QAbstractItemView
)
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QColor, QBrush
import os
import json
import yaml
from pathlib import Path

class BenchmarkPage(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setObjectName("benchmarkPage")
        self.init_ui()
        
    def init_ui(self):
        layout = QVBoxLayout(self)
        layout.setContentsMargins(30, 30, 30, 30)
        
        # Header
        header_layout = QHBoxLayout()
        title = QLabel("Global Benchmark")
        title.setStyleSheet("font-size: 24px; font-weight: bold; color: #f8fafc;")
        header_layout.addWidget(title)
        
        header_layout.addStretch()
        
        self.btn_refresh = QPushButton("Refresh Data")
        self.btn_refresh.setObjectName("actionButton")
        self.btn_refresh.setFixedWidth(150)
        self.btn_refresh.clicked.connect(self.refresh_data)
        header_layout.addWidget(self.btn_refresh)
        
        layout.addLayout(header_layout)
        
        # Table
        self.table = QTableWidget()
        self.table.setColumnCount(8)
        self.table.setHorizontalHeaderLabels([
            "Run ID", "SLAM", "Dataset", "Duration", 
            "ATE (m)", "Coverage (%)", "IoU", "Path (m)"
        ])
        self.table.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeToContents)
        self.table.horizontalHeader().setSectionResizeMode(1, QHeaderView.Stretch)
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
        
    def open_result_window(self, item):
        row = item.row()
        run_id = self.table.item(row, 0).text()
        
        # Reconstruct path
        root = Path.cwd()
        run_dir = root / "results" / "runs" / run_id
        
        if run_dir.exists():
            from gui.results_window import ResultWindow
            # We need to keep a reference to prevent GC
            # If parent is MainWindow, we can use it?
            # Or just attach to self.
            if not hasattr(self, "result_windows"):
                self.result_windows = []
                
            try:
                # Clean up closed windows from list
                self.result_windows = [w for w in self.result_windows if w.isVisible()]
                
                win = ResultWindow(run_dir, self)
                win.show()
                self.result_windows.append(win)
                
                # If metrics are computed/saved when window opens, we might want to refresh table?
                # But window computation is asyncish? No it's synchronous in init (in my code).
                # So we can refresh immediately after?
                # win runs analysis in init.
                self.refresh_data()
                
            except Exception as e:
                print(f"Error opening results: {e}")
        
    def refresh_data(self):
        root = Path.cwd()
        runs_dir = root / "results" / "runs"
        
        if not runs_dir.exists():
            return

        runs = []
        
        # Scan folders
        for run_path in runs_dir.iterdir():
            if run_path.is_dir():
                run_data = self.parse_run(run_path)
                if run_data:
                    runs.append(run_data)
        
        # Sort by timestamp (run_id key) desc
        runs.sort(key=lambda x: x['id'], reverse=True)
        
        self.populate_table(runs)
        
    def parse_run(self, path: Path):
        data = {
            "id": path.name,
            "slam": "N/A",
            "dataset": "N/A",
            "duration": None,
            "ate": None,
            "coverage": None,
            "iou": None,
            "path": None
        }
        
        # Config
        config_path = path / "config_resolved.yaml"
        if config_path.exists():
            try:
                with open(config_path, 'r') as f:
                    cfg = yaml.safe_load(f)
                    # SLAM used?
                    # structure: includes: [{config: slam_path, ...}]
                    # Ideally config_resolved might have flattened structure or we check the name
                    # Let's try to infer from run name for now or check 'slams' key if present
                    # or look for "slam_toolbox", "gmapping" in inclusions.
                    # Usually configured via include options.
                    
                    # Parse from Run ID: DATE_TIME_DATASET_SLAM_SEED_R0
                    parts = path.name.split("__")
                    if len(parts) >= 2:
                        data["dataset"] = parts[1]
                    if len(parts) >= 3:
                        data["slam"] = parts[2]
            except:
                pass
                
        # Metrics
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
            except:
                pass
                
        return data

    def populate_table(self, runs):
        self.table.setRowCount(0)
        self.table.setRowCount(len(runs))
        
        for r, run in enumerate(runs):
            # ID
            self.table.setItem(r, 0, QTableWidgetItem(run["id"]))
            
            # SLAM
            self.table.setItem(r, 1, QTableWidgetItem(run["slam"]))
            
            # Dataset
            self.table.setItem(r, 2, QTableWidgetItem(run["dataset"]))
            
            # Duration
            dur_val = run.get("duration")
            item_dur = QTableWidgetItem(f"{dur_val:.1f} s" if dur_val is not None else "-")
            if dur_val is not None:
                item_dur.setTextAlignment(Qt.AlignRight | Qt.AlignVCenter)
            self.table.setItem(r, 3, item_dur)
            
            # ATE
            ate_val = run["ate"]
            item_ate = QTableWidgetItem(f"{ate_val:.3f}" if ate_val is not None else "-")
            if ate_val is not None:
                item_ate.setTextAlignment(Qt.AlignRight | Qt.AlignVCenter)
                # Color coding? Lower is better.
                if ate_val < 0.1: item_ate.setForeground(QColor("#4ade80")) # Green
                elif ate_val > 1.0: item_ate.setForeground(QColor("#f87171")) # Red
            self.table.setItem(r, 4, item_ate)
            
            # Coverage
            cov_val = run["coverage"]
            item_cov = QTableWidgetItem(f"{cov_val:.1f}%" if cov_val is not None else "-")
            if cov_val is not None:
                item_cov.setTextAlignment(Qt.AlignRight | Qt.AlignVCenter)
                if cov_val > 95: item_cov.setForeground(QColor("#4ade80"))
            self.table.setItem(r, 5, item_cov)
            
            # IoU
            iou_val = run["iou"]
            item_iou = QTableWidgetItem(f"{iou_val:.3f}" if iou_val is not None else "-")
            if iou_val is not None:
                item_iou.setTextAlignment(Qt.AlignRight | Qt.AlignVCenter)
                if iou_val > 0.8: item_iou.setForeground(QColor("#4ade80"))
            self.table.setItem(r, 6, item_iou)
            
            # Path
            path_val = run["path"]
            item_path = QTableWidgetItem(f"{path_val:.1f}" if path_val is not None else "-")
            if path_val is not None:
                item_path.setTextAlignment(Qt.AlignRight | Qt.AlignVCenter)
            self.table.setItem(r, 7, item_path)
