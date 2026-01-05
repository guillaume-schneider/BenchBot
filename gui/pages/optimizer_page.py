from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton, 
    QComboBox, QSpinBox, QDoubleSpinBox, QTableWidget, QTableWidgetItem,
    QHeaderView, QFileDialog, QMessageBox, QFrame, QScrollArea, QLineEdit, QGroupBox
)
from PyQt5.QtCore import Qt, QThread, pyqtSignal
from PyQt5.QtGui import QColor
import yaml
import json
from pathlib import Path

class OptimizerThread(QThread):
    progress = pyqtSignal(str)
    finished = pyqtSignal(dict)
    
    def __init__(self, base_job, tuning_config, trials, name):
        super().__init__()
        self.base_job = base_job
        self.tuning_config = tuning_config
        self.trials = trials
        self.name = name
        
    def run(self):
        from tools.optimizer import SLAMOptimizer
        try:
            opt = SLAMOptimizer(self.base_job, self.tuning_config, self.name)
            study = opt.run(n_trials=self.trials)
            self.finished.emit({
                "best_value": study.best_value,
                "best_params": study.best_params
            })
        except Exception as e:
            import traceback
            self.progress.emit(f"Error: {e}\n{traceback.format_exc()}")

class OptimizerPage(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.init_ui()
        self.params_to_tune = []

    def init_ui(self):
        layout = QVBoxLayout(self)
        layout.setContentsMargins(30, 30, 30, 30)
        layout.setSpacing(15)
        
        # Header
        header = QLabel("SLAM Auto-Tuner (Optuna)")
        header.setStyleSheet("font-size: 24px; font-weight: bold; color: #f8fafc;")
        layout.addWidget(header)
        
        subheader = QLabel("Automatically optimize SLAM parameters to minimize trajectory error.")
        subheader.setStyleSheet("color: #94a3b8; margin-bottom: 10px;")
        layout.addWidget(subheader)

        # Style for common elements
        self.setStyleSheet("""
            QLabel { color: #e2e8f0; }
            QGroupBox { 
                font-weight: bold; 
                color: #818cf8; 
                border: 1px solid #334155; 
                border-radius: 8px; 
                margin-top: 15px;
                padding-top: 15px;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 10px;
                padding: 0 5px;
            }
            QLineEdit, QComboBox, QSpinBox, QDoubleSpinBox {
                background-color: #0f172a;
                color: #f8fafc;
                border: 1px solid #475569;
                padding: 6px;
                border-radius: 4px;
            }
            QTableWidget {
                background-color: #0f172a;
                color: #e2e8f0;
                gridline-color: #334155;
                border-radius: 4px;
            }
            QHeaderView::section {
                background-color: #1e293b;
                color: #94a3b8;
                padding: 5px;
                border: none;
            }
            QPushButton#actionButton {
                background-color: #6366f1;
                color: white;
                font-weight: bold;
                border-radius: 4px;
                padding: 8px;
            }
            QPushButton#actionButton:hover { background-color: #4f46e5; }
            QPushButton#secondaryButton {
                background-color: #334155;
                color: #f8fafc;
                border-radius: 4px;
                padding: 6px;
            }
            QPushButton#secondaryButton:hover { background-color: #475569; }
        """)

        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setStyleSheet("background: transparent; border: none;")
        content = QWidget()
        self.content_layout = QVBoxLayout(content)
        scroll.setWidget(content)
        layout.addWidget(scroll)

        # 1. Base Configuration
        base_card = QGroupBox("1. Setup Base Run")
        base_layout = QVBoxLayout(base_card)
        
        row1 = QHBoxLayout()
        row1.addWidget(QLabel("Reference Job (YAML):"))
        self.base_job_path = QLineEdit()
        self.base_job_path.setPlaceholderText("Select a resolved job from results/jobs/...")
        btn_browse = QPushButton("Browse")
        btn_browse.setObjectName("secondaryButton")
        btn_browse.clicked.connect(self.browse_base_job)
        row1.addWidget(self.base_job_path)
        row1.addWidget(btn_browse)
        base_layout.addLayout(row1)
        self.content_layout.addWidget(base_card)

        # 2. Parameters
        param_card = QGroupBox("2. Optimization Search Space")
        self.param_layout = QVBoxLayout(param_card)
        
        preset_layout = QHBoxLayout()
        preset_layout.addWidget(QLabel("Add Common Param:"))
        self.combo_presets = QComboBox()
        self.combo_presets.addItems([
            "--- GMapping ---",
            "slam.parameters.slam_gmapping.maxUrange",
            "slam.parameters.slam_gmapping.map_update_interval",
            "slam.parameters.slam_gmapping.particles",
            "--- Cartographer ---",
            "slam.parameters.num_accumulated_range_data",
            "slam.parameters.max_range"
        ])
        btn_add = QPushButton("Add Parameter")
        btn_add.setObjectName("secondaryButton")
        btn_add.clicked.connect(self.add_param_from_preset)
        preset_layout.addWidget(self.combo_presets)
        preset_layout.addWidget(btn_add)
        self.param_layout.addLayout(preset_layout)

        self.params_table = QTableWidget()
        self.params_table.setColumnCount(4)
        self.params_table.setHorizontalHeaderLabels(["Parameter Path", "Min", "Max", "Actions"])
        self.params_table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.params_table.setFixedHeight(200)
        self.param_layout.addWidget(self.params_table)
        self.content_layout.addWidget(param_card)

        # 3. Execution Settings
        exec_card = QGroupBox("3. Optimization Strategy")
        exec_layout = QHBoxLayout(exec_card)
        
        exec_layout.addWidget(QLabel("Number of Trials:"))
        self.spin_trials = QSpinBox()
        self.spin_trials.setRange(2, 100)
        self.spin_trials.setValue(10)
        exec_layout.addWidget(self.spin_trials)
        
        exec_layout.addStretch()
        
        self.btn_start = QPushButton("Run Optimization")
        self.btn_start.setObjectName("actionButton")
        self.btn_start.setFixedWidth(200)
        self.btn_start.clicked.connect(self.start_optimization)
        exec_layout.addWidget(self.btn_start)
        self.content_layout.addWidget(exec_card)
        
        self.status_label = QLabel("Ready.")
        self.status_label.setStyleSheet("color: #10b981; font-weight: bold; margin-top: 10px;")
        layout.addWidget(self.status_label)

    def browse_base_job(self):
        f, _ = QFileDialog.getOpenFileName(self, "Select Job Configuration", "results/jobs", "YAML (*.yaml)")
        if f:
            self.base_job_path.setText(f)

    def add_param_from_preset(self):
        path = self.combo_presets.currentText()
        if "---" in path: return
        self.add_param_row(path, 0.0, 10.0)

    def add_param_row(self, path, vmin, vmax):
        row = self.params_table.rowCount()
        self.params_table.insertRow(row)
        
        self.params_table.setItem(row, 0, QTableWidgetItem(path))
        
        spin_min = QDoubleSpinBox()
        spin_min.setRange(-1000, 10000)
        spin_min.setValue(vmin)
        self.params_table.setCellWidget(row, 1, spin_min)
        
        spin_max = QDoubleSpinBox()
        spin_max.setRange(-1000, 10000)
        spin_max.setValue(vmax)
        self.params_table.setCellWidget(row, 2, spin_max)
        
        btn_del = QPushButton("Remove")
        btn_del.setObjectName("secondaryButton")
        btn_del.clicked.connect(lambda: self.params_table.removeRow(self.params_table.currentRow()))
        self.params_table.setCellWidget(row, 3, btn_del)

    def start_optimization(self):
        base_job = self.base_job_path.text()
        if not base_job:
            QMessageBox.warning(self, "Input Missing", "Please select a reference job.")
            return

        if self.params_table.rowCount() == 0:
            QMessageBox.warning(self, "Input Missing", "Please add at least one parameter to tune.")
            return

        tuning_spec = {"params": {}}
        for r in range(self.params_table.rowCount()):
            path = self.params_table.item(r, 0).text()
            vmin = self.params_table.cellWidget(r, 1).value()
            vmax = self.params_table.cellWidget(r, 2).value()
            tuning_spec["params"][path] = {
                "type": "float",
                "min": vmin,
                "max": vmax
            }
        
        tuning_path = Path("results/optimization/temp_space.yaml")
        tuning_path.parent.mkdir(parents=True, exist_ok=True)
        with open(tuning_path, "w") as f:
            yaml.dump(tuning_spec, f)
            
        self.btn_start.setEnabled(False)
        self.btn_start.setText("Optimizer Running...")
        self.status_label.setText("Optimization in progress... Check terminal for logs.")
        
        self.thread = OptimizerThread(base_job, str(tuning_path), self.spin_trials.value(), "gui_optimizer")
        self.thread.finished.connect(self.on_finished)
        self.thread.progress.connect(lambda m: self.status_label.setText(m))
        self.thread.start()

    def load_reference_job(self, path):
        self.base_job_path.setText(path)
        # Optional: auto-add some common params based on the SLAM type in the job
        try:
            with open(path, 'r') as f:
                data = yaml.safe_load(f)
                slam_id = data.get("slam", {}).get("id", "").lower()
                if "gmapping" in slam_id:
                    self.add_param_row("slam.parameters.slam_gmapping.maxUrange", 5.0, 20.0)
                    self.add_param_row("slam.parameters.slam_gmapping.particles", 10, 80)
                elif "toolbox" in slam_id:
                    self.add_param_row("slam.parameters.max_range", 5.0, 30.0)
        except:
            pass

    def on_finished(self, results):
        self.btn_start.setEnabled(True)
        self.btn_start.setText("Run Optimization")
        self.status_label.setText(f"Optimization Finished. Best RMSE: {results['best_value']:.4f}")
        
        res_str = f"Best Results Found:\n\nValue (ATE RMSE): {results['best_value']:.4f}\n\nParameters:\n"
        for k, v in results["best_params"].items():
            res_str += f"  - {k}: {v}\n"
            
        QMessageBox.information(self, "Optimization Complete", res_str)
