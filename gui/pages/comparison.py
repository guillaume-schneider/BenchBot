from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton, 
    QComboBox, QFrame, QTableWidget, QTableWidgetItem, QHeaderView, QFileDialog, QMessageBox
)
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QFont
from pathlib import Path
import json
import yaml
import numpy as np
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg
from matplotlib.figure import Figure

try:
    from evaluation import (
        load_gt_map, read_messages_by_topic, get_trajectory, compute_coverage
    )
    EVALUATION_AVAILABLE = True
except ImportError:
    EVALUATION_AVAILABLE = False

class ComparisonPage(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.layout = QVBoxLayout(self)
        self.layout.setContentsMargins(30, 30, 30, 30)
        self.layout.setSpacing(20)
        
        self.init_ui()
        self.last_runs_data = []
        
    def init_ui(self):
        # 1. Header
        header = QLabel("Benchmark Comparison")
        header.setStyleSheet("font-size: 24px; font-weight: bold; color: #f8fafc;")
        self.layout.addWidget(header)
        
        # 2. Selection Row
        selection_frame = QFrame()
        selection_frame.setStyleSheet("background-color: #1e293b; border-radius: 8px; border: 1px solid #334155;")
        sel_layout = QHBoxLayout(selection_frame)
        
        self.combos = []
        colors = ["#3b82f6", "#ef4444", "#10b981"] # blue, red, green
        
        for i in range(3):
            vbox = QVBoxLayout()
            lbl = QLabel(f"Run {i+1}")
            lbl.setStyleSheet(f"color: {colors[i]}; font-weight: bold; border: none;")
            combo = QComboBox()
            combo.setFixedWidth(300)
            combo.setStyleSheet("""
                QComboBox { background-color: #0f172a; border: 1px solid #475569; padding: 8px; border-radius: 6px; color: white; }
                QComboBox::drop-down { border: none; }
            """)
            vbox.addWidget(lbl)
            vbox.addWidget(combo)
            sel_layout.addLayout(vbox)
            self.combos.append(combo)
            
        compare_btn = QPushButton("Compare Runs")
        compare_btn.setFixedHeight(45)
        compare_btn.setFixedWidth(150)
        compare_btn.setStyleSheet("""
            QPushButton { background-color: #6366f1; color: white; border: none; border-radius: 6px; font-weight: bold; font-size: 14px; margin-top: 15px; }
            QPushButton:hover { background-color: #4f46e5; }
        """)
        compare_btn.setCursor(Qt.PointingHandCursor)
        compare_btn.clicked.connect(self.run_comparison)
        sel_layout.addWidget(compare_btn)
        
        refresh_btn = QPushButton("Refresh List")
        refresh_btn.setFixedHeight(45)
        refresh_btn.setFixedWidth(120)
        refresh_btn.setStyleSheet("""
            QPushButton { background-color: #334155; color: white; border: none; border-radius: 6px; font-weight: bold; margin-top: 15px; }
            QPushButton:hover { background-color: #475569; }
        """)
        refresh_btn.clicked.connect(self.scan_runs)
        sel_layout.addWidget(refresh_btn)
        
        self.export_btn = QPushButton("Export PDF Report")
        self.export_btn.setFixedHeight(45)
        self.export_btn.setFixedWidth(160)
        self.export_btn.setEnabled(False)
        self.export_btn.setStyleSheet("""
            QPushButton { background-color: #059669; color: white; border: none; border-radius: 6px; font-weight: bold; margin-top: 15px; }
            QPushButton:hover { background-color: #047857; }
            QPushButton:disabled { background-color: #334155; color: #94a3b8; }
        """)
        self.export_btn.clicked.connect(self.export_report)
        sel_layout.addWidget(self.export_btn)
        
        sel_layout.addStretch()
        
        self.layout.addWidget(selection_frame)
        
        # 3. Main Content (Plot & Table)
        main_h_layout = QHBoxLayout()
        
        # Plot Frame
        plot_frame = QFrame()
        plot_frame.setStyleSheet("background-color: #1e293b; border-radius: 8px; border: 1px solid #334155;")
        plot_vbox = QVBoxLayout(plot_frame)
        
        self.figure = Figure(figsize=(8, 8), facecolor='#1e293b')
        self.canvas = FigureCanvasQTAgg(self.figure)
        self.ax = self.figure.add_subplot(111)
        self.ax.set_facecolor('#0f172a')
        self.ax.tick_params(colors='#94a3b8', labelsize=8)
        self.ax.grid(True, color='#334155', linestyle='--', alpha=0.5)
        plot_vbox.addWidget(self.canvas)
        
        main_h_layout.addWidget(plot_frame, 3) # 60% width
        
        # Table Frame
        table_frame = QFrame()
        table_frame.setStyleSheet("background-color: #1e293b; border-radius: 8px; border: 1px solid #334155;")
        table_vbox = QVBoxLayout(table_frame)
        
        table_title = QLabel("Metrics Comparison")
        table_title.setStyleSheet("color: #f8fafc; font-weight: bold; padding: 10px; border: none; border-bottom: 1px solid #334155;")
        table_vbox.addWidget(table_title)
        
        self.table = QTableWidget()
        self.table.setColumnCount(3)
        self.table.setHorizontalHeaderLabels(["Run 1", "Run 2", "Run 3"])
        self.table.setVerticalHeaderLabels([
            "Health Status",
            "SLAM (Algo)", 
            "Dataset (Scenario)", 
            "ATE (Abs. Traj. Error m)", 
            "Coverage (% Area)", 
            "Max RAM (Peak MB)",
            "Max CPU (Peak %)"
        ])
        self.table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.table.verticalHeader().setStyleSheet("color: #94a3b8; font-weight: bold;")
        self.table.setStyleSheet("""
            QTableWidget { background: transparent; border: none; color: #e2e8f0; gridline-color: #334155; }
            QHeaderView::section { background-color: #0f172a; color: #94a3b8; padding: 10px; border: none; font-weight: bold; }
        """)
        table_vbox.addWidget(self.table)
        
        main_h_layout.addWidget(table_frame, 2) # 40% width
        
        self.layout.addLayout(main_h_layout)
        
        self.scan_runs()

    def scan_runs(self):
        root_path = Path("results/runs")
        if not root_path.exists():
            return
            
        runs = sorted([d for d in root_path.iterdir() if d.is_dir()], reverse=True)
        
        for combo in self.combos:
            combo.clear()
            combo.addItem("-- Select Run --", None)
            for r in runs:
                # Preview name: timestamp + slam
                parts = r.name.split("__")
                display = r.name
                if len(parts) >= 3:
                    display = f"{parts[0]} - {parts[2]} ({parts[1]})"
                combo.addItem(display, str(r))

    def run_comparison(self):
        if not EVALUATION_AVAILABLE:
            return

        self.ax.clear()
        self.ax.set_facecolor('#0f172a')
        self.ax.grid(True, color='#334155', linestyle='--', alpha=0.5)
        self.table.setRowCount(7)
        
        colors = ["#3b82f6", "#ef4444", "#10b981"]
        gt_loaded = False
        runs_to_report = []
        
        for i, combo in enumerate(self.combos):
            run_path_str = combo.currentData()
            if not run_path_str:
                for row in range(7):
                    self.table.setItem(row, i, QTableWidgetItem("-"))
                continue
                
            run_path = Path(run_path_str)
            metrics = {}
            if (run_path / "metrics.json").exists():
                with open(run_path / "metrics.json") as f:
                    metrics = json.load(f)
            
            # 1. Info extraction from name
            parts = run_path.name.split("__")
            dataset = parts[1] if len(parts) > 1 else "?"
            slam = parts[2] if len(parts) > 2 else "?"
            
            # 2. Metrics implementation
            ate = metrics.get('ate_rmse', 0.0)
            ram = metrics.get('max_ram_mb', 0.0)
            cpu = metrics.get('max_cpu_percent', 0.0)
            cov = metrics.get('coverage', 0.0)
            
            # 7. Status & Failure detection (NOW AT ROW 0)
            is_failure = metrics.get('is_failure', False)
            reasons = metrics.get('failure_reasons', [])
            
            if is_failure:
                status_item = QTableWidgetItem("❌ ANOMALY [ℹ️ details]")
                status_item.setForeground(Qt.red)
                status_item.setFont(QFont("Segoe UI", 9, QFont.Bold))
                status_item.setToolTip("ANOMALIES DETECTED (Hover for details):\n\n" + "\n".join([f"• {r}" for r in reasons]))
            else:
                status_item = QTableWidgetItem("✅ VALID RUN")
                status_item.setForeground(Qt.green)
            
            self.table.setItem(0, i, status_item)
            self.table.setItem(1, i, QTableWidgetItem(slam))
            self.table.setItem(2, i, QTableWidgetItem(dataset))
            self.table.setItem(3, i, QTableWidgetItem(f"{ate:.4f} m" if ate else "N/A"))
            self.table.setItem(4, i, QTableWidgetItem(f"{cov*100:.1f} %" if cov else "N/A"))
            self.table.setItem(5, i, QTableWidgetItem(f"{ram:.1f} MB" if ram else "N/A"))
            self.table.setItem(6, i, QTableWidgetItem(f"{cpu:.1f} %" if cpu else "N/A"))

            # Store for PDF report
            runs_to_report.append({
                'name': run_path.name,
                'slam': slam,
                'dataset': dataset,
                'ate': f"{ate:.4f}" if ate else "N/A",
                'coverage': f"{cov*100:.1f}" if cov else "N/A",
                'ram': f"{ram:.1f}" if ram else "N/A",
                'cpu': f"{cpu:.1f}" if cpu else "N/A",
                'status': "❌ ANOMALY" if is_failure else "✅ VALID",
                'is_failure': is_failure,
                'reasons': reasons
            })

            # 3. Trajectory extraction
            bag_dir = run_path / "bags" / "output"
            if bag_dir.exists():
                try:
                    msgs = read_messages_by_topic(str(bag_dir), ["/odom"])
                    tx, ty = get_trajectory(msgs.get("/odom", []))
                    self.ax.plot(tx, ty, color=colors[i], label=f"Run {i+1} ({slam})", linewidth=2)
                except Exception as e:
                    print(f"Error loading trajectory for {run_path.name}: {e}")

            # 4. Load GT Map background (only once)
            if not gt_loaded:
                resolved_cfg = run_path / "config_resolved.yaml"
                if resolved_cfg.exists():
                    try:
                        with open(resolved_cfg) as f:
                            cfg = yaml.safe_load(f)
                            gt_def = cfg.get("dataset", {}).get("ground_truth", {})
                            if gt_def:
                                full_gt = (Path.cwd() / gt_def.get("map_path")).resolve()
                                if full_gt.exists():
                                    gt_map, gt_res, gt_origin = load_gt_map(str(full_gt))
                                    extents = [
                                        gt_origin[0], 
                                        gt_origin[0] + gt_map.shape[1] * gt_res,
                                        gt_origin[1],
                                        gt_origin[1] + gt_map.shape[0] * gt_res
                                    ]
                                    vis = np.zeros(gt_map.shape)
                                    vis[gt_map == 0] = 0.8
                                    vis[gt_map > 50] = 0.2
                                    self.ax.imshow(np.flipud(vis), extent=extents, origin='lower', cmap='gray', alpha=0.3)
                                    gt_loaded = True
                    except:
                        pass
        
        self.ax.legend(facecolor='#1e293b', edgecolor='#334155', labelcolor='white')
        self.ax.set_title("Trajectory Comparison Overlay", color='white')
        self.ax.relim()
        self.ax.autoscale_view()
        self.canvas.draw()
        
        # Save data for report
        self.last_runs_data = runs_to_report
        self.export_btn.setEnabled(len(self.last_runs_data) > 0)

    def export_report(self):
        if not self.last_runs_data:
            return
            
        file_path, _ = QFileDialog.getSaveFileName(self, "Export Comparison Report", "comparison_report.pdf", "PDF Files (*.pdf)")
        if not file_path:
            return
            
        try:
            # 1. Save current plot to temp image
            temp_plot = Path("/tmp/comparison_plot.png")
            self.figure.savefig(str(temp_plot), dpi=150, facecolor=self.figure.get_facecolor())
            
            # 2. Generate PDF
            from tools.report_generator import generate_full_report
            generate_full_report(file_path, self.last_runs_data, str(temp_plot))
            
            # 3. Success notification
            QMessageBox.information(self, "Export Success", f"Report successfully generated at:\n{file_path}")
            
            # Cleanup
            if temp_plot.exists():
                temp_plot.unlink()
                
        except Exception as e:
            QMessageBox.critical(self, "Export Error", f"Failed to generate report: {str(e)}")
