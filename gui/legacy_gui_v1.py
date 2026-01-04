import sys
import os
import yaml
import json
import asyncio
import subprocess
import shutil
import signal
import selectors
import time
from pathlib import Path
from datetime import datetime
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, 
    QPushButton, QLabel, QTextEdit, QTableWidget, 
    QTableWidgetItem, QHeaderView, QFileDialog, QMessageBox,
    QProgressBar, QGroupBox, QScrollArea, QLineEdit, QCheckBox,
    QFrame, QListWidget, QListWidgetItem, QInputDialog, QStackedWidget,
    QSpacerItem, QSizePolicy, QFormLayout, QComboBox
)
from PyQt5.QtCore import Qt, QThread, pyqtSignal, QTimer, QRegExp, QSize
from PyQt5.QtGui import QFont, QColor, QPalette, QSyntaxHighlighter, QTextCharFormat, QPainter, QLinearGradient

# Add project root to sys.path
PROJECT_ROOT = Path(__file__).parent.parent
sys.path.append(str(PROJECT_ROOT))

from runner.resolve import load_yaml, resolve_run_config, stable_run_id, write_yaml
from generate_gt_map import generate_map
from show_map import show_map

# --- Styling (Premium Glassmorphism / Dark Mode) ---
STYLE_SHEET = """
QMainWindow, QWidget#mainScreen {
    background-color: #0f172a;
    color: #f8fafc;
    font-family: 'Inter', 'Segoe UI', Roboto, Helvetica, Arial, sans-serif;
}

/* Sidebar Styling */
QWidget#sidebar {
    background-color: #1e293b;
    border-right: 1px solid #334155;
}

QListWidget#navList {
    background-color: transparent;
    border: none;
    outline: none;
    padding-top: 20px;
}

QListWidget#navList::item {
    color: #94a3b8;
    padding: 15px 25px;
    border-radius: 8px;
    margin: 4px 10px;
    font-size: 14px;
    font-weight: 500;
}

QListWidget#navList::item:selected {
    background-color: #334155;
    color: #f8fafc;
}

QListWidget#navList::item:hover:!selected {
    background-color: #1e293b;
    color: #cbd5e1;
}

/* Card Styling */
QFrame[class="card"] {
    background-color: rgba(30, 41, 59, 0.7);
    border: 1px solid #334155;
    border-radius: 12px;
}

QLabel#headerLabel {
    font-size: 24px;
    font-weight: 700;
    color: #f8fafc;
    margin-bottom: 20px;
}

QLabel#cardTitle {
    font-size: 16px;
    font-weight: 600;
    color: #f8fafc;
    margin-bottom: 5px;
}

/* Form Elements */
QTextEdit, QLineEdit, QListWidget#configList {
    background-color: #0f172a;
    border: 1px solid #334155;
    color: #f1f5f9;
    padding: 10px;
    border-radius: 8px;
}

QPushButton {
    background-color: #6366f1;
    color: white;
    border: none;
    padding: 10px 20px;
    border-radius: 8px;
    font-weight: 600;
}

QPushButton:hover {
    background-color: #4f46e5;
}

QPushButton#cancelButton {
    background-color: #334155;
    color: #f1f5f9;
}

QPushButton#cancelButton:hover {
    background-color: #475569;
}

QPushButton#actionButton {
    background-color: #6366f1;
}

QPushButton#dangerButton {
    background-color: #ef4444;
}

/* Progress Bar */
QProgressBar {
    background-color: #0f172a;
    border: 1px solid #334155;
    border-radius: 10px;
    height: 20px;
    text-align: center;
    color: transparent;
}

QProgressBar::chunk {
    background: qlineargradient(x1:0, y1:0, x2:1, y2:0, stop:0 #6366f1, stop:1 #a855f7);
    border-radius: 10px;
}

/* GroupBox and Forms in Visual Editor */
QGroupBox#visualSection {
    background-color: rgba(30, 41, 59, 0.4);
    border: 1px solid #334155;
    border-radius: 12px;
    margin-top: 25px;
    padding: 20px;
    color: #f8fafc;
    font-size: 16px;
    font-weight: 700;
}

QGroupBox#visualSection::title {
    subcontrol-origin: margin;
    left: 15px;
    padding: 0 10px;
}

/* Ensure labels are visible (not black) */
VisualConfigEditor QLabel {
    color: #cbd5e1;
    font-weight: 600;
}

VisualConfigEditor QLineEdit, VisualConfigEditor QComboBox {
    background-color: #0f172a;
    border: 1px solid #334155;
    color: #f1f5f9;
    padding: 8px 12px;
    border-radius: 6px;
}

VisualConfigEditor QPushButton {
    margin-top: 5px;
}

/* Table */
QTableWidget {
    background-color: transparent;
    border: none;
    gridline-color: #334155;
    color: #f1f5f9;
}

QHeaderView::section {
    background-color: #1e293b;
    color: #94a3b8;
    padding: 8px;
    border: none;
    font-weight: 600;
}
"""

class YamlHighlighter(QSyntaxHighlighter):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.rules = []
        key_format = QTextCharFormat()
        key_format.setForeground(QColor("#818cf8"))
        key_format.setFontWeight(QFont.Bold)
        self.rules.append((QRegExp(r"^\s*[\w.-]+(?=\s*:)"), key_format))
        self.rules.append((QRegExp(r"(?<=- )\s*[\w.-]+(?=\s*:)"), key_format))
        value_format = QTextCharFormat()
        value_format.setForeground(QColor("#34d399"))
        self.rules.append((QRegExp(r":\s+\".*\""), value_format))
        self.rules.append((QRegExp(r":\s+'.*'"), value_format))
        num_format = QTextCharFormat()
        num_format.setForeground(QColor("#fbbf24"))
        self.rules.append((QRegExp(r":\s+(true|false|null|\d+(\.\d+)?)"), num_format))
        comment_format = QTextCharFormat()
        comment_format.setForeground(QColor("#64748b"))
        self.rules.append((QRegExp(r"#.*"), comment_format))

    def highlightBlock(self, text):
        for expression, format in self.rules:
            index = expression.indexIn(text)
            while index >= 0:
                length = expression.matchedLength()
                self.setFormat(index, length, format)
                index = expression.indexIn(text, index + length)

class RunWorker(QThread):
    log_signal = pyqtSignal(str)
    progress_signal = pyqtSignal(int, int, str)
    finished_signal = pyqtSignal()

    def __init__(self, configs_paths, use_gui):
        super().__init__()
        self.configs_paths = configs_paths
        self.use_gui = use_gui
        self.is_cancelled = False

    def run(self):
        try:
            total_resolved_jobs = []
            for matrix_path in self.configs_paths:
                matrix = load_yaml(matrix_path)
                output_root = matrix.get("output", {}).get("root_dir", "results/runs")
                Path(output_root).mkdir(parents=True, exist_ok=True)
                slams_map = {s["id"]: s for s in matrix.get("slams", [])}
                datasets_map = {d["id"]: d for d in matrix.get("datasets", [])}
                for inc in matrix.get("matrix", {}).get("include", []):
                    d_id = inc["dataset"]
                    dataset_def = datasets_map.get(d_id)
                    if not dataset_def: continue
                    for s_id in inc.get("slams", []):
                        slam_entry = slams_map.get(s_id)
                        if not slam_entry: continue
                        profile_path = PROJECT_ROOT / slam_entry["profile"]
                        slam_profile = load_yaml(profile_path)
                        for seed in inc.get("seeds", [0]):
                            for r in range(inc.get("repeats", 1)):
                                run_id = stable_run_id(d_id, s_id, seed, r)
                                resolved = resolve_run_config(
                                    matrix=matrix, dataset_obj=dataset_def,
                                    slam_entry=slam_entry, slam_profile=slam_profile,
                                    combo_overrides=inc.get("overrides"),
                                    slam_overrides=slam_entry.get("overrides"),
                                    dataset_overrides=dataset_def.get("overrides"),
                                    seed=seed, repeat_index=r, run_id=run_id, output_root=output_root
                                )
                                if self.use_gui:
                                    for proc in resolved.get("dataset", {}).get("scenario", {}).get("processes", []):
                                        if proc.get("name") == "nav2_sim":
                                            cmd = proc.get("cmd", [])
                                            new_cmd = [arg.replace("headless:=True", "headless:=False").replace("gui:=False", "gui:=True") for arg in cmd]
                                            proc["cmd"] = new_cmd
                                config_path = Path(output_root) / run_id / "config_resolved.yaml"
                                total_resolved_jobs.append((run_id, config_path, resolved))

            total = len(total_resolved_jobs)
            for i, (run_id, config_path, resolved) in enumerate(total_resolved_jobs):
                if self.is_cancelled: break
                self.progress_signal.emit(i + 1, total, run_id)
                self.log_signal.emit(f"INFO: Starting benchmark {run_id} ({i+1}/{total})...")
                write_yaml(config_path, resolved)
                
                cmd = [sys.executable, "-m", "runner.run_one", str(config_path)]
                
                # Start in a new process group to allow killing the entire tree
                process = subprocess.Popen(
                    cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, 
                    cwd=str(PROJECT_ROOT), universal_newlines=True,
                    preexec_fn=os.setsid
                )
                
                # Use selectors for non-blocking I/O to check is_cancelled frequently
                sel = selectors.DefaultSelector()
                sel.register(process.stdout, selectors.EVENT_READ)
                
                try:
                    while True:
                        if self.is_cancelled:
                            self.log_signal.emit("WARN: Cancellation requested. Terminating process group...")
                            os.killpg(os.getpgid(process.pid), signal.SIGTERM)
                            break
                        
                        events = sel.select(timeout=0.1)
                        if events:
                            line = process.stdout.readline()
                            if not line: # EOF
                                break
                            self.log_signal.emit(line.strip())
                        
                        if process.poll() is not None:
                            # Final flush of remaining output
                            for line in process.stdout:
                                self.log_signal.emit(line.strip())
                            break
                finally:
                    sel.unregister(process.stdout)
                    sel.close()
                
                process.wait()
                if self.is_cancelled:
                    self.log_signal.emit(f"CANCELLED: {run_id}")
                    break
                
                if process.returncode == 0:
                    self.log_signal.emit(f"SUCCESS: {run_id} complete.")
                else:
                    self.log_signal.emit(f"FAILURE: {run_id} failed with code {process.returncode}.")
        except Exception as e:
            self.log_signal.emit(f"ERROR: {str(e)}")
        finally:
            self.finished_signal.emit()

class VisualConfigEditor(QScrollArea):
    data_changed = pyqtSignal()

    def __init__(self):
        super().__init__()
        self.setWidgetResizable(True)
        self.container = QWidget()
        self.layout = QVBoxLayout(self.container)
        self.setWidget(self.container)
        self.setStyleSheet("background-color: transparent; border: none;")

        self.sections = {}
        self.init_ui()

    def init_ui(self):
        self.layout.setSpacing(20)
        self.layout.setContentsMargins(15, 15, 25, 15)

        # General Section
        self.general_group = QGroupBox("General")
        self.general_group.setObjectName("visualSection")
        gen_layout = QFormLayout(self.general_group)
        gen_layout.setSpacing(15)
        gen_layout.setContentsMargins(15, 25, 15, 15)
        self.name_edit = QLineEdit()
        self.schema_edit = QLineEdit()
        gen_layout.addRow("Name:", self.name_edit)
        gen_layout.addRow("Schema Version:", self.schema_edit)
        self.layout.addWidget(self.general_group)

        # Workspace Section
        self.ws_group = QGroupBox("Workspace")
        self.ws_group.setObjectName("visualSection")
        ws_layout = QFormLayout(self.ws_group)
        ws_layout.setSpacing(15)
        ws_layout.setContentsMargins(15, 25, 15, 15)
        self.ros_distro = QLineEdit()
        self.use_sim_time = QCheckBox("Use Sim Time")
        self.use_sim_time.setStyleSheet("color: #f8fafc; font-weight: bold;")
        ws_layout.addRow("ROS Distro:", self.ros_distro)
        ws_layout.addRow(self.use_sim_time)
        self.layout.addWidget(self.ws_group)

        # Defaults Section
        self.defaults_group = QGroupBox("Defaults (Benchmark Params)")
        self.defaults_group.setObjectName("visualSection")
        def_layout = QFormLayout(self.defaults_group)
        def_layout.setSpacing(15)
        def_layout.setContentsMargins(15, 25, 15, 15)
        self.warmup_edit = QLineEdit()
        self.timeout_edit = QLineEdit()
        self.rosbag_topics = QLineEdit()
        def_layout.addRow("Warmup (s):", self.warmup_edit)
        def_layout.addRow("Timeout (s):", self.timeout_edit)
        def_layout.addRow("Rosbag Topics:", self.rosbag_topics)
        self.layout.addWidget(self.defaults_group)

        # Datasets Section
        self.datasets_group = QGroupBox("Datasets")
        self.datasets_group.setObjectName("visualSection")
        self.datasets_layout = QVBoxLayout(self.datasets_group)
        self.datasets_layout.setContentsMargins(15, 25, 15, 15)
        add_ds_btn = QPushButton("+ Add Dataset")
        add_ds_btn.setFixedWidth(200)
        add_ds_btn.setObjectName("actionButton")
        add_ds_btn.clicked.connect(lambda: self.add_dataset_item({}))
        self.datasets_layout.addWidget(add_ds_btn)
        self.layout.addWidget(self.datasets_group)

        # SLAMs Section
        self.slams_group = QGroupBox("SLAM Profiles")
        self.slams_group.setObjectName("visualSection")
        self.slams_layout = QVBoxLayout(self.slams_group)
        self.slams_layout.setContentsMargins(15, 25, 15, 15)
        add_slam_btn = QPushButton("+ Add SLAM Profile")
        add_slam_btn.setFixedWidth(200)
        add_slam_btn.setObjectName("actionButton")
        add_slam_btn.clicked.connect(lambda: self.add_slam_item({}))
        self.slams_layout.addWidget(add_slam_btn)
        self.layout.addWidget(self.slams_group)

        # Matrix Section
        self.matrix_group = QGroupBox("Execution Matrix (Include)")
        self.matrix_group.setObjectName("visualSection")
        self.matrix_layout = QVBoxLayout(self.matrix_group)
        self.matrix_layout.setContentsMargins(15, 25, 15, 15)
        add_inc_btn = QPushButton("+ Add Include Entry")
        add_inc_btn.setFixedWidth(200)
        add_inc_btn.setObjectName("actionButton")
        add_inc_btn.clicked.connect(lambda: self.add_include_item({}))
        self.matrix_layout.addWidget(add_inc_btn)
        self.layout.addWidget(self.matrix_group)

        self.layout.addStretch()

    def add_dataset_item(self, data):
        item_widget = QFrame()
        item_widget.setProperty("class", "card")
        item_widget.setStyleSheet("background-color: rgba(15, 23, 42, 0.6); margin-bottom: 10px; border: 1px dashed #475569;")
        l = QVBoxLayout(item_widget)
        f = QFormLayout()
        f.setSpacing(10)
        id_edit = QLineEdit(data.get("id", ""))
        kind_edit = QLineEdit(data.get("kind", "sim_gazebo"))
        f.addRow("ID:", id_edit)
        f.addRow("Kind:", kind_edit)
        l.addLayout(f)
        rem_btn = QPushButton("Remove Dataset", objectName="dangerButton")
        rem_btn.clicked.connect(lambda: item_widget.deleteLater())
        l.addWidget(rem_btn)
        self.datasets_layout.insertWidget(self.datasets_layout.count() - 1, item_widget)

    def add_slam_item(self, data):
        item_widget = QFrame()
        item_widget.setProperty("class", "card")
        item_widget.setStyleSheet("background-color: rgba(15, 23, 42, 0.6); margin-bottom: 10px; border: 1px dashed #475569;")
        l = QVBoxLayout(item_widget)
        f = QFormLayout()
        f.setSpacing(10)
        id_edit = QLineEdit(data.get("id", ""))
        profile_edit = QLineEdit(data.get("profile", ""))
        f.addRow("ID:", id_edit)
        f.addRow("Profile Path:", profile_edit)
        l.addLayout(f)
        rem_btn = QPushButton("Remove SLAM", objectName="dangerButton")
        rem_btn.clicked.connect(lambda: item_widget.deleteLater())
        l.addWidget(rem_btn)
        self.slams_layout.insertWidget(self.slams_layout.count() - 1, item_widget)

    def add_include_item(self, data):
        item_widget = QFrame()
        item_widget.setProperty("class", "card")
        item_widget.setStyleSheet("background-color: rgba(15, 23, 42, 0.6); margin-bottom: 10px; border: 1px dashed #475569;")
        l = QVBoxLayout(item_widget)
        
        f = QFormLayout()
        f.setSpacing(10)
        ds_edit = QLineEdit(data.get("dataset", ""))
        slams_edit = QLineEdit(", ".join(data.get("slams", [])))
        seeds_edit = QLineEdit(", ".join(map(str, data.get("seeds", [0]))))
        repeats_edit = QLineEdit(str(data.get("repeats", 1)))
        
        f.addRow("Dataset ID:", ds_edit)
        f.addRow("SLAM IDs:", slams_edit)
        f.addRow("Seeds:", seeds_edit)
        f.addRow("Repeats:", repeats_edit)
        l.addLayout(f)

        rem_btn = QPushButton("Remove Entry", objectName="dangerButton")
        rem_btn.clicked.connect(lambda: item_widget.deleteLater())
        l.addWidget(rem_btn)
        
        # Insert before the "Add" button
        self.matrix_layout.insertWidget(self.matrix_layout.count() - 1, item_widget)

    def load_data(self, data):
        # Clear existing dynamic items
        for layout in [self.matrix_layout, self.datasets_layout, self.slams_layout]:
            for i in reversed(range(layout.count())):
                w = layout.itemAt(i).widget()
                if isinstance(w, QFrame):
                    w.deleteLater()

        self.name_edit.setText(str(data.get("name", "")))
        self.schema_edit.setText(str(data.get("schema_version", "1")))
        
        ws = data.get("workspace", {})
        self.ros_distro.setText(ws.get("ros_distro", "humble"))
        self.use_sim_time.setChecked(ws.get("use_sim_time", True))
        
        defs = data.get("defaults", {})
        run_defs = defs.get("run", {})
        self.warmup_edit.setText(str(run_defs.get("warmup_s", 2.0)))
        self.timeout_edit.setText(str(run_defs.get("timeout_s", 90.0)))
        
        bag_defs = defs.get("rosbag", {})
        self.rosbag_topics.setText(", ".join(bag_defs.get("topics", [])))
        
        for ds in data.get("datasets", []):
            self.add_dataset_item(ds)
        for s in data.get("slams", []):
            self.add_slam_item(s)
        for inc in data.get("matrix", {}).get("include", []):
            self.add_include_item(inc)

    def get_data(self):
        data = {
            "name": self.name_edit.text(),
            "schema_version": int(self.schema_edit.text() or 1),
            "workspace": {
                "ros_distro": self.ros_distro.text(),
                "use_sim_time": self.use_sim_time.isChecked()
            },
            "defaults": {
                "run": {
                    "warmup_s": float(self.warmup_edit.text() or 0.0),
                    "timeout_s": float(self.timeout_edit.text() or 0.0)
                },
                "rosbag": {
                    "topics": [t.strip() for t in self.rosbag_topics.text().split(",") if t.strip()]
                }
            },
            "datasets": [],
            "slams": [],
            "matrix": {"include": []}
        }
        
        # Datasets
        for i in range(self.datasets_layout.count()):
            w = self.datasets_layout.itemAt(i).widget()
            if isinstance(w, QFrame):
                f_layout = w.layout().itemAt(0)
                data["datasets"].append({
                    "id": f_layout.itemAt(0, QFormLayout.FieldRole).widget().text(),
                    "kind": f_layout.itemAt(1, QFormLayout.FieldRole).widget().text()
                })
        # SLAMs
        for i in range(self.slams_layout.count()):
            w = self.slams_layout.itemAt(i).widget()
            if isinstance(w, QFrame):
                f_layout = w.layout().itemAt(0)
                data["slams"].append({
                    "id": f_layout.itemAt(0, QFormLayout.FieldRole).widget().text(),
                    "profile": f_layout.itemAt(1, QFormLayout.FieldRole).widget().text()
                })
        # Matrix
        for i in range(self.matrix_layout.count()):
            w = self.matrix_layout.itemAt(i).widget()
            if isinstance(w, QFrame):
                f_layout = w.layout().itemAt(0)
                ds = f_layout.itemAt(0, QFormLayout.FieldRole).widget().text()
                slams = [s.strip() for s in f_layout.itemAt(1, QFormLayout.FieldRole).widget().text().split(",") if s.strip()]
                seeds_str = f_layout.itemAt(2, QFormLayout.FieldRole).widget().text().split(",")
                seeds = [int(s.strip()) for s in seeds_str if s.strip().isdigit()]
                repeats = int(f_layout.itemAt(3, QFormLayout.FieldRole).widget().text() or 1)
                
                data["matrix"]["include"].append({
                    "dataset": ds,
                    "slams": slams,
                    "seeds": seeds,
                    "repeats": repeats
                })
        return data

class GTMapPage(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.layout = QVBoxLayout(self)
        self.layout.setContentsMargins(40, 40, 40, 40)
        self.init_ui()

    def init_ui(self):
        header = QLabel("Ground Truth Map Generator")
        header.setObjectName("headerLabel")
        self.layout.addWidget(header)

        self.form_card = QFrame()
        self.form_card.setProperty("class", "card")
        self.form_card.setObjectName("visualSection")
        form_layout = QFormLayout(self.form_card)
        form_layout.setSpacing(15)
        form_layout.setContentsMargins(20, 25, 20, 20)

        # SDF Path
        sdf_row = QHBoxLayout()
        self.sdf_path = QLineEdit("world/model.sdf")
        sdf_row.addWidget(self.sdf_path)
        browse_btn = QPushButton("Browse")
        browse_btn.setFixedWidth(80)
        browse_btn.clicked.connect(self.browse_sdf)
        sdf_row.addWidget(browse_btn)
        form_layout.addRow("Gazebo Model (SDF):", sdf_row)

        # Params
        self.res = QLineEdit("0.05")
        self.laser_z = QLineEdit("0.17")
        self.padding = QLineEdit("1.5")
        self.out_name = QLineEdit("map_gt")
        
        form_layout.addRow("Resolution (m/px):", self.res)
        form_layout.addRow("Laser Height (m):", self.laser_z)
        form_layout.addRow("Padding (m):", self.padding)
        form_layout.addRow("Output Base Name:", self.out_name)

        # Options
        self.gen_png = QCheckBox("Generate PNG image")
        self.gen_png.setChecked(True)
        self.gen_png.setStyleSheet("color: #f8fafc;")
        self.gen_debug = QCheckBox("Generate Debug geometry plot")
        self.gen_debug.setChecked(True)
        self.gen_debug.setStyleSheet("color: #f8fafc;")
        form_layout.addRow(self.gen_png)
        form_layout.addRow(self.gen_debug)

        self.layout.addWidget(self.form_card)

        # Actions
        btn_row = QHBoxLayout()
        gen_btn = QPushButton("Generate GT Map", objectName="actionButton")
        gen_btn.clicked.connect(self.run_generation)
        btn_row.addWidget(gen_btn)

        preview_btn = QPushButton("Preview Result")
        preview_btn.clicked.connect(self.preview_map)
        btn_row.addWidget(preview_btn)

        debug_btn = QPushButton("Show Debug Plot")
        debug_btn.clicked.connect(self.preview_debug)
        btn_row.addWidget(debug_btn)
        
        btn_row.addStretch()
        self.layout.addLayout(btn_row)
        self.layout.addStretch()

    def browse_sdf(self):
        path, _ = QFileDialog.getOpenFileName(self, "Select SDF Model", "", "SDF Files (*.sdf);;All Files (*)")
        if path:
            self.sdf_path.setText(path)

    def run_generation(self):
        try:
            success, msg = generate_map(
                sdf_path=self.sdf_path.text(),
                resolution=float(self.res.text()),
                laser_z=float(self.laser_z.text()),
                padding=float(self.padding.text()),
                output_name=self.out_name.text(),
                gen_png=self.gen_png.isChecked(),
                gen_debug=self.gen_debug.isChecked()
            )
            if success:
                QMessageBox.information(self, "Success", msg)
            else:
                QMessageBox.critical(self, "Error", msg)
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Failed to start generation: {str(e)}")

    def preview_map(self):
        yaml_path = f"{self.out_name.text()}.yaml"
        if Path(yaml_path).exists():
            try:
                show_map(yaml_path)
            except Exception as e:
                QMessageBox.critical(self, "Viewer Error", str(e))
        else:
            QMessageBox.warning(self, "Missing File", "Generate the map first.")

    def preview_debug(self):
        debug_path = f"{self.out_name.text()}_debug.png"
        if Path(debug_path).exists():
            # Simply use OS default viewer or a matplotlib figure
            import matplotlib.pyplot as plt
            import matplotlib.image as mpimg
            try:
                img = mpimg.imread(debug_path)
                plt.figure("Debug Geometry Plot")
                plt.imshow(img)
                plt.axis('off')
                plt.show()
            except Exception as e:
                QMessageBox.critical(self, "Viewer Error", str(e))
        else:
            QMessageBox.warning(self, "Missing File", "Generate the debug plot first.")

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("SLAM Bench Orchestrator")
        self.resize(1200, 850)
        self.setStyleSheet(STYLE_SHEET)

        self.matrices_dir = PROJECT_ROOT / "configs/matrices"
        self.matrices_dir.mkdir(parents=True, exist_ok=True)

        self.central_widget = QWidget()
        self.central_widget.setObjectName("mainScreen")
        self.setCentralWidget(self.central_widget)
        
        self.layout = QHBoxLayout(self.central_widget)
        self.layout.setContentsMargins(0, 0, 0, 0)
        self.layout.setSpacing(0)

        self.init_sidebar()
        self.init_main_content()

        self.worker = None

    def init_sidebar(self):
        sidebar = QWidget()
        sidebar.setObjectName("sidebar")
        sidebar.setFixedWidth(240)
        sidebar_layout = QVBoxLayout(sidebar)
        sidebar_layout.setContentsMargins(0, 20, 0, 20)

        # Logo / Text
        logo = QLabel("SLAM Bench")
        logo.setStyleSheet("font-size: 20px; font-weight: bold; color: #f8fafc; padding: 20px 25px;")
        sidebar_layout.addWidget(logo)

        self.nav_list = QListWidget()
        self.nav_list.setObjectName("navList")
        self.nav_list.addItem(QListWidgetItem("⊞ Dashboard"))
        self.nav_list.addItem(QListWidgetItem("⚙ Configurations"))
        self.nav_list.addItem(QListWidgetItem("🗺 GT Map Generator"))
        self.nav_list.addItem(QListWidgetItem("📊 Benchmarks"))
        self.nav_list.addItem(QListWidgetItem("📈 Results"))
        self.nav_list.addItem(QListWidgetItem("⚒ Settings"))
        
        self.nav_list.setCurrentRow(0)
        self.nav_list.currentRowChanged.connect(self.switch_page)
        
        sidebar_layout.addWidget(self.nav_list)
        sidebar_layout.addStretch()
        
        self.layout.addWidget(sidebar)

    def init_main_content(self):
        self.stack = QStackedWidget()
        
        self.page_dashboard = self.create_dashboard_page()
        self.page_configs = self.create_configs_page()
        self.page_gt_map = GTMapPage()
        self.page_results = self.create_results_page()
        self.page_placeholder = QLabel("Content coming soon...")
        self.page_placeholder.setAlignment(Qt.AlignCenter)
        self.page_placeholder.setStyleSheet("color: #94a3b8; font-size: 18px;")

        self.stack.addWidget(self.page_dashboard)
        self.stack.addWidget(self.page_configs)
        self.stack.addWidget(self.page_gt_map)
        self.stack.addWidget(self.page_placeholder) # Benchmarks
        self.stack.addWidget(self.page_results)
        self.stack.addWidget(self.page_placeholder) # Settings

        self.layout.addWidget(self.stack)

    def create_dashboard_page(self):
        page = QWidget()
        layout = QVBoxLayout(page)
        layout.setContentsMargins(40, 40, 40, 40)

        header = QLabel("Dashboard")
        header.setObjectName("headerLabel")
        layout.addWidget(header)

        # Top Grid: Configs and Logs
        top_grid = QHBoxLayout()
        
        # Selected Configs Card
        self.card_configs = QFrame()
        self.card_configs.setObjectName("configsCard")
        self.card_configs.setProperty("class", "card")
        config_layout = QVBoxLayout(self.card_configs)
        config_layout.addWidget(QLabel("Active Configurations", objectName="cardTitle"))
        
        self.dash_config_list = QListWidget()
        self.dash_config_list.setObjectName("dashConfigList")
        self.dash_config_list.setStyleSheet("background: transparent; border: none;")
        config_layout.addWidget(self.dash_config_list)
        
        start_batch_btn = QPushButton("Start Selected Batch", objectName="actionButton")
        start_batch_btn.clicked.connect(self.start_batch_run)
        config_layout.addWidget(start_batch_btn)
        
        self.gui_cb = QCheckBox("Enable Gazebo GUI")
        self.gui_cb.setStyleSheet("color: #94a3b8; margin-top: 5px;")
        config_layout.addWidget(self.gui_cb)
        
        top_grid.addWidget(self.card_configs, 1)

        # Logs Card
        self.card_logs = QFrame()
        self.card_logs.setProperty("class", "card")
        log_layout = QVBoxLayout(self.card_logs)
        log_layout.addWidget(QLabel("Live Console Log", objectName="cardTitle"))
        self.log_view = QTextEdit()
        self.log_view.setReadOnly(True)
        self.log_view.setFont(QFont("Monospace", 10))
        self.log_view.setStyleSheet("background-color: #0f172a; border-radius: 8px; color: #34d399;")
        log_layout.addWidget(self.log_view)
        
        top_grid.addWidget(self.card_logs, 2)
        layout.addLayout(top_grid, 3)

        # Bottom: Progress Card
        self.card_progress = QFrame()
        self.card_progress.setProperty("class", "card")
        self.card_progress.setFixedHeight(180)
        prog_layout = QVBoxLayout(self.card_progress)
        
        self.prog_title = QLabel("Benchmarking: Idle", objectName="cardTitle")
        prog_layout.addWidget(self.prog_title)
        
        prog_inner = QHBoxLayout()
        self.progress_bar = QProgressBar()
        prog_inner.addWidget(self.progress_bar, 5)
        
        self.stop_btn = QPushButton("Cancel", objectName="cancelButton")
        self.stop_btn.setFixedWidth(100)
        self.stop_btn.setEnabled(False)
        self.stop_btn.clicked.connect(self.stop_batch_run)
        prog_inner.addWidget(self.stop_btn)
        prog_layout.addLayout(prog_inner)
        
        self.status_label = QLabel("Waiting to start...")
        self.status_label.setStyleSheet("color: #94a3b8;")
        prog_layout.addWidget(self.status_label)

        layout.addWidget(self.card_progress, 1)
        return page

    def create_configs_page(self):
        page = QWidget()
        layout = QVBoxLayout(page)
        layout.setContentsMargins(40, 40, 40, 40)

        header = QLabel("Configurations")
        header.setObjectName("headerLabel")
        layout.addWidget(header)

        content = QHBoxLayout()
        
        # Left sidebar in page
        list_panel = QVBoxLayout()
        self.config_list = QListWidget()
        self.config_list.setObjectName("configList")
        self.config_list.itemClicked.connect(self.on_config_selected)
        list_panel.addWidget(self.config_list)
        
        actions = QHBoxLayout()
        new_btn = QPushButton("New")
        new_btn.clicked.connect(self.new_config)
        actions.addWidget(new_btn)
        clone_btn = QPushButton("Clone")
        clone_btn.clicked.connect(self.clone_config)
        actions.addWidget(clone_btn)
        list_panel.addLayout(actions)
        
        delete_btn = QPushButton("Delete Selected", objectName="dangerButton")
        delete_btn.clicked.connect(self.delete_config)
        list_panel.addWidget(delete_btn)
        
        content.addLayout(list_panel, 1)

        # Right editor
        editor_panel = QVBoxLayout()
        header_row = QHBoxLayout()
        self.editor_label = QLabel("Select a file to edit")
        self.editor_label.setStyleSheet("color: #94a3b8;")
        header_row.addWidget(self.editor_label)
        
        header_row.addStretch()
        header_row.addWidget(QLabel("Mode:"))
        self.mode_selector = QComboBox()
        self.mode_selector.addItems(["YAML Mode", "Visual Mode"])
        self.mode_selector.currentIndexChanged.connect(self.toggle_editor_mode)
        header_row.addWidget(self.mode_selector)
        editor_panel.addLayout(header_row)
        
        self.editor_stack = QStackedWidget()
        
        # YAML Mode
        self.config_editor = QTextEdit()
        self.config_editor.setFont(QFont("Monospace", 11))
        self.highlighter = YamlHighlighter(self.config_editor.document())
        self.editor_stack.addWidget(self.config_editor)
        
        # Visual Mode
        self.visual_editor = VisualConfigEditor()
        self.editor_stack.addWidget(self.visual_editor)
        
        editor_panel.addWidget(self.editor_stack)
        
        self.save_btn = QPushButton("Save Changes")
        self.save_btn.setEnabled(False)
        self.save_btn.clicked.connect(self.save_config)
        editor_panel.addWidget(self.save_btn)
        
        content.addLayout(editor_panel, 3)
        layout.addLayout(content)
        
        self.refresh_config_list()
        return page

    def create_results_page(self):
        page = QWidget()
        layout = QVBoxLayout(page)
        layout.setContentsMargins(40, 40, 40, 40)

        header = QLabel("Results")
        header.setObjectName("headerLabel")
        layout.addWidget(header)

        self.results_table = QTableWidget(0, 4)
        self.results_table.setHorizontalHeaderLabels(["Run ID", "SLAM", "Dataset", "ATE RMSE"])
        self.results_table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        layout.addWidget(self.results_table)

        refresh_btn = QPushButton("Refresh Data")
        refresh_btn.clicked.connect(self.load_results)
        layout.addWidget(refresh_btn)
        
        self.load_results()
        return page

    def switch_page(self, index):
        self.stack.setCurrentIndex(index)
        if index == 0: # Dashboard
             self.update_dash_list()

    def refresh_config_list(self):
        self.config_list.clear()
        for f in sorted(self.matrices_dir.glob("*.yaml")):
            item = QListWidgetItem(f.name)
            item.setFlags(item.flags() | Qt.ItemIsUserCheckable | Qt.ItemIsSelectable | Qt.ItemIsEnabled)
            item.setCheckState(Qt.Unchecked)
            self.config_list.addItem(item)
        self.update_dash_list()

    def update_dash_list(self):
        self.dash_config_list.clear()
        selected_count = 0
        for i in range(self.config_list.count()):
            item = self.config_list.item(i)
            if item.checkState() == Qt.Checked:
                dash_item = QListWidgetItem(f"✓ {item.text()}")
                dash_item.setForeground(QColor("#818cf8"))
                self.dash_config_list.addItem(dash_item)
                selected_count += 1
        
        if selected_count == 0:
            self.dash_config_list.addItem("No configurations selected.")

    def on_config_selected(self, item):
        self.current_config_path = self.matrices_dir / item.text()
        self.editor_label.setText(f"Path: {self.current_config_path.relative_to(PROJECT_ROOT)}")
        with open(self.current_config_path, "r") as f:
            self.config_editor.setPlainText(f.read())
        self.save_btn.setEnabled(True)
        self.update_dash_list()

    def new_config(self):
        name, ok = QInputDialog.getText(self, "New Config", "Name (.yaml):")
        if ok and name:
            if not name.endswith(".yaml"): name += ".yaml"
            new_path = self.matrices_dir / name
            template = self.matrices_dir / "default.yaml"
            if template.exists(): shutil.copy(template, new_path)
            else:
                with open(new_path, "w") as f:
                    f.write("# New Configuration\nschema_version: 1\nname: \"unnamed_config\"\n")
            self.refresh_config_list()

    def clone_config(self):
        item = self.config_list.currentItem()
        if not item: return
        name, ok = QInputDialog.getText(self, "Clone Config", "New name:", text=f"copy_{item.text()}")
        if ok and name:
            if not name.endswith(".yaml"): name += ".yaml"
            shutil.copy(self.matrices_dir / item.text(), self.matrices_dir / name)
            self.refresh_config_list()

    def toggle_editor_mode(self, index):
        if not hasattr(self, 'current_config_path'): return
        
        if index == 1: # Switching TO Visual Mode
            try:
                content = self.config_editor.toPlainText()
                data = yaml.safe_load(content) or {}
                self.visual_editor.load_data(data)
                self.editor_stack.setCurrentIndex(1)
            except Exception as e:
                QMessageBox.warning(self, "YAML Error", f"Cannot switch to Visual Mode: {str(e)}")
                self.mode_selector.setCurrentIndex(0)
        else: # Switching TO YAML Mode
            data = self.visual_editor.get_data()
            # We want to preserve existing YAML structure (probes, datasets lists etc.)
            # For now, we'll merge back into the full original text if possible, 
            # or just dump the whole thing.
            try:
                original_content = self.config_editor.toPlainText()
                original_data = yaml.safe_load(original_content) or {}
                # Update top-level keys managed by Visual Editor
                managed_data = self.visual_editor.get_data()
                original_data.update(managed_data)
                
                new_yaml = yaml.dump(original_data, sort_keys=False)
                self.config_editor.setPlainText(new_yaml)
                self.editor_stack.setCurrentIndex(0)
            except Exception as e:
                QMessageBox.critical(self, "Save Error", f"Error syncing data: {str(e)}")

    def delete_config(self):
        item = self.config_list.currentItem()
        if not item or item.text() == "default.yaml": return
        confirm = QMessageBox.question(self, "Delete", f"Permanently delete {item.text()}?", QMessageBox.Yes|QMessageBox.No)
        if confirm == QMessageBox.Yes:
            os.remove(self.matrices_dir / item.text())
            self.refresh_config_list()
            self.config_editor.clear()

    def save_config(self):
        if not hasattr(self, 'current_config_path'): return
        try:
            if self.editor_stack.currentIndex() == 1:
                # Sync back to YAML first
                self.toggle_editor_mode(0)
                self.mode_selector.setCurrentIndex(0)
            
            content = self.config_editor.toPlainText()
            yaml.safe_load(content)
            with open(self.current_config_path, "w") as f:
                f.write(content)
            QMessageBox.information(self, "Saved", "Config successfully updated.")
        except Exception as e:
            QMessageBox.critical(self, "Error", str(e))

    def load_results(self):
        results_dir = PROJECT_ROOT / "results/runs"
        if not results_dir.exists(): return
        self.results_table.setRowCount(0)
        for d in sorted(results_dir.iterdir(), reverse=True):
            if d.is_dir():
                metrics_path = d / "metrics.json"
                config_path = d / "config_resolved.yaml"
                if config_path.exists():
                    cfg = load_yaml(config_path)
                    ate = "N/A"
                    if metrics_path.exists():
                        try:
                            with open(metrics_path, "r") as f:
                                metrics = json.load(f)
                                ate_val = metrics.get('ate_rmse')
                                if ate_val is not None:
                                    ate = f"{float(ate_val):.4f}"
                        except:
                            pass
                    row = self.results_table.rowCount()
                    self.results_table.insertRow(row)
                    self.results_table.setItem(row, 0, QTableWidgetItem(d.name))
                    self.results_table.setItem(row, 1, QTableWidgetItem(cfg["slam"]["id"]))
                    self.results_table.setItem(row, 2, QTableWidgetItem(cfg["dataset"]["id"]))
                    self.results_table.setItem(row, 3, QTableWidgetItem(ate))

    def start_batch_run(self):
        selected_paths = []
        for i in range(self.config_list.count()):
            item = self.config_list.item(i)
            if item.checkState() == Qt.Checked:
                selected_paths.append(str(self.matrices_dir / item.text()))
        if not selected_paths:
            QMessageBox.warning(self, "Selection", "Select at least one config in Configurations tab.")
            return

        self.log_view.clear()
        self.stop_btn.setEnabled(True)
        self.worker = RunWorker(selected_paths, self.gui_cb.isChecked())
        self.worker.log_signal.connect(self.append_log)
        self.worker.progress_signal.connect(self.update_progress)
        self.worker.finished_signal.connect(self.on_run_finished)
        self.worker.start()

    def stop_batch_run(self):
        if self.worker:
            self.worker.is_cancelled = True
            self.append_log("WARN: User requested cancellation. Stopping immediately...")

    def update_progress(self, current, total, run_id):
        self.progress_bar.setMaximum(total)
        self.progress_bar.setValue(current)
        self.prog_title.setText(f"Benchmarking: {run_id}")
        self.status_label.setText(f"Batch progress: {current} / {total} jobs completed.")

    def append_log(self, text):
        self.log_view.append(text)
        self.log_view.ensureCursorVisible()

    def on_run_finished(self):
        self.stop_btn.setEnabled(False)
        self.prog_title.setText("Benchmarking: idle")
        self.status_label.setText("Batch run finished.")
        self.load_results()

if __name__ == "__main__":
    # Handle high DPI displays
    if hasattr(Qt, 'AA_EnableHighDpiScaling'):
        QApplication.setAttribute(Qt.AA_EnableHighDpiScaling, True)
    if hasattr(Qt, 'AA_UseHighDpiPixmaps'):
        QApplication.setAttribute(Qt.AA_UseHighDpiPixmaps, True)

    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())
