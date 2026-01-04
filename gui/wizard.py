from PyQt5.QtWidgets import (
    QDialog, QVBoxLayout, QHBoxLayout, QLabel, QPushButton, QStackedWidget,
    QLineEdit, QFormLayout, QComboBox, QFileDialog, QMessageBox, QCheckBox,
    QListWidget, QListWidgetItem, QGroupBox, QRadioButton, QButtonGroup, QFrame
)
from PyQt5.QtCore import Qt, pyqtSignal
from pathlib import Path
import yaml
import shutil

class ConfigWizard(QDialog):
    config_created = pyqtSignal()
    
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Create Configuration")
        self.resize(600, 500)
        self.setStyleSheet("""
            QDialog { background-color: #0f172a; color: #f8fafc; }
            QLabel { color: #f8fafc; font-size: 14px; }
            QLineEdit, QComboBox { 
                background-color: #1e293b; border: 1px solid #334155; 
                padding: 8px; border-radius: 6px; color: #f8fafc; 
            }
            QPushButton {
                background-color: #3b82f6; color: white; border: none; padding: 8px 16px; border-radius: 6px; font-weight: bold;
            }
            QPushButton:hover { background-color: #2563eb; }
            QPushButton#secondary { background-color: #334155; }
            QPushButton#secondary:hover { background-color: #475569; }
            QListWidget { background-color: #1e293b; border: 1px solid #334155; border-radius: 6px; color: #f8fafc; }
        """)
        
        self.layout = QVBoxLayout(self)
        self.stack = QStackedWidget()
        self.layout.addWidget(self.stack)
        
        # Pages
        self.page_mode = self.create_mode_page()
        self.page_basic = self.create_basic_page()
        self.page_dataset = self.create_dataset_page()
        self.page_slams = self.create_slams_page()
        self.page_import = self.create_import_page()
        
        self.stack.addWidget(self.page_mode)     # 0
        self.stack.addWidget(self.page_basic)    # 1
        self.stack.addWidget(self.page_dataset)  # 2
        self.stack.addWidget(self.page_slams)    # 3
        self.stack.addWidget(self.page_import)   # 4
        
        # Data
        self.data = {
            "name": "",
            "datasets": [],
            "slams": []
        }

    def create_page_container(self, title):
        widget = QWidget()
        layout = QVBoxLayout(widget)
        
        lbl_title = QLabel(title)
        lbl_title.setStyleSheet("font-size: 20px; font-weight: bold; margin-bottom: 20px; color: #60a5fa;")
        layout.addWidget(lbl_title)
        
        content = QWidget()
        layout.addWidget(content)
        layout.addStretch()
        
        btn_layout = QHBoxLayout()
        layout.addLayout(btn_layout)
        
        return widget, content, btn_layout

    def create_mode_page(self):
        widget, content, btns = self.create_page_container("How would you like to start?")
        cl = QVBoxLayout(content)
        cl.setSpacing(15)
        
        self.btn_guided = QPushButton("Guided Wizard\nCreate a new configuration step-by-step")
        self.btn_guided.setFixedSize(400, 80)
        self.btn_guided.clicked.connect(lambda: self.stack.setCurrentIndex(1))
        
        self.btn_import = QPushButton("Import File\nImport an existing YAML configuration")
        self.btn_import.setFixedSize(400, 80)
        self.btn_import.clicked.connect(lambda: self.stack.setCurrentIndex(4))
        
        # Styling larger buttons
        style = """
            QPushButton { text-align: left; padding: 15px; font-size: 15px; }
        """
        self.btn_guided.setStyleSheet(style)
        self.btn_import.setStyleSheet(style)
        
        cl.addWidget(self.btn_guided, alignment=Qt.AlignCenter)
        cl.addWidget(self.btn_import, alignment=Qt.AlignCenter)
        
        return widget

    def create_basic_page(self):
        widget, content, btns = self.create_page_container("Step 1: Basic Information")
        form = QFormLayout(content)
        
        self.inp_name = QLineEdit()
        self.inp_name.setPlaceholderText("e.g. experiment_01")
        form.addRow("Configuration Name:", self.inp_name)
        
        self.inp_desc = QLineEdit()
        form.addRow("Description (Optional):", self.inp_desc)
        
        # Nav
        b_next = QPushButton("Next")
        b_next.clicked.connect(self.validate_basic)
        btns.addStretch()
        btns.addWidget(b_next)
        
        return widget

    def create_dataset_page(self):
        widget, content, btns = self.create_page_container("Step 2: Dataset Configuration")
        l = QVBoxLayout(content)
        
        # We allow creating ONE dataset for simplicity in wizard, could expand later
        form = QFormLayout()
        
        self.ds_id = QLineEdit("tb3_sim")
        form.addRow("Dataset ID:", self.ds_id)
        
        world_row = QHBoxLayout()
        self.ds_world = QLineEdit("worlds/turtlebot3_house.world")
        b_world = QPushButton("Browse")
        b_world.setFixedWidth(60)
        b_world.clicked.connect(lambda: self.browse_file(self.ds_world, "World (*.world *.sdf)"))
        world_row.addWidget(self.ds_world)
        world_row.addWidget(b_world)
        form.addRow("World File:", world_row)
        
        gt_row = QHBoxLayout()
        self.ds_gt = QLineEdit()
        self.ds_gt.setPlaceholderText("Optional: generated_maps/map.yaml")
        b_gt = QPushButton("Browse")
        b_gt.setFixedWidth(60)
        b_gt.clicked.connect(lambda: self.browse_file(self.ds_gt, "Map (*.yaml)"))
        gt_row.addWidget(self.ds_gt)
        gt_row.addWidget(b_gt)
        form.addRow("GT Map (Auto-gen if SDF):", gt_row)
        
        l.addLayout(form)
        
        # Nav
        b_back = QPushButton("Back")
        b_back.setObjectName("secondary")
        b_back.clicked.connect(lambda: self.stack.setCurrentIndex(1))
        
        b_next = QPushButton("Next")
        b_next.clicked.connect(self.validate_dataset)
        
        btns.addWidget(b_back)
        btns.addStretch()
        btns.addWidget(b_next)
        
        return widget

    def create_slams_page(self):
        widget, content, btns = self.create_page_container("Step 3: Select Algorithms")
        l = QVBoxLayout(content)
        
        self.slam_list = QListWidget()
        self.slam_list.setSelectionMode(QListWidget.MultiSelection)
        
        # Populate
        slams_dir = Path("configs/slams")
        if slams_dir.exists():
            for f in sorted(slams_dir.glob("*.yaml")):
                item = QListWidgetItem(f.stem)
                item.setData(Qt.UserRole, str(f))
                self.slam_list.addItem(item)
                
        l.addWidget(QLabel("Select SLAM algorithms to include:"))
        l.addWidget(self.slam_list)
        
        # Nav
        b_back = QPushButton("Back")
        b_back.setObjectName("secondary")
        b_back.clicked.connect(lambda: self.stack.setCurrentIndex(2))
        
        b_finish = QPushButton("Finish & Create")
        b_finish.setStyleSheet("background-color: #22c55e;")
        b_finish.clicked.connect(self.finish_wizard)
        
        btns.addWidget(b_back)
        btns.addStretch()
        btns.addWidget(b_finish)
        
        return widget
        
    def create_import_page(self):
        widget, content, btns = self.create_page_container("Import Configuration")
        l = QVBoxLayout(content)
        
        row = QHBoxLayout()
        self.imp_path = QLineEdit()
        b_br = QPushButton("Browse")
        b_br.clicked.connect(lambda: self.browse_file(self.imp_path, "YAML (*.yaml)"))
        row.addWidget(self.imp_path)
        row.addWidget(b_br)
        
        l.addWidget(QLabel("Select existing configuration file:"))
        l.addLayout(row)
        l.addWidget(QLabel("This file will be copied to your configs/matrices folder."))
        
        b_import = QPushButton("Import")
        b_import.setStyleSheet("background-color: #22c55e;")
        b_import.clicked.connect(self.run_import)
        
        btns.addStretch()
        btns.addWidget(b_import)
        
        return widget

    def browse_file(self, line_edit, filter):
        f, _ = QFileDialog.getOpenFileName(self, "Select File", "", filter)
        if f: line_edit.setText(f)

    def validate_basic(self):
        if not self.inp_name.text().strip():
            QMessageBox.warning(self, "Missing Name", "Please enter a configuration name.")
            return
        self.data["name"] = self.inp_name.text().strip()
        self.stack.setCurrentIndex(2)

    def validate_dataset(self):
        if not self.ds_id.text().strip():
             QMessageBox.warning(self, "Missing ID", "Dataset ID required.")
             return
             
        ds = {
            "id": self.ds_id.text().strip(),
            "world_name": self.ds_world.text().strip()
            # Launch params could be inferred or default
        }
        
        if self.ds_gt.text().strip():
            ds["ground_truth"] = {"map_path": self.ds_gt.text().strip()}
        
        # Simple default launch command for now if not specified
        # In a real wizard we might want more options
        ds["launch"] = {"cmd": "ros2 launch nav2_bringup tb3_simulation_launch.py headless:=True world:={world_name}"}
        
        self.data["datasets"] = [ds] 
        self.stack.setCurrentIndex(3)

    def finish_wizard(self):
        # Gather SLAMs
        selected = self.slam_list.selectedItems()
        if not selected:
             QMessageBox.warning(self, "Missing SLAM", "Please select at least one SLAM algorithm.")
             return
             
        slams = []
        for item in selected:
            slams.append({"id": item.text(), "config_path": item.data(Qt.UserRole)})
        
        self.data["slams"] = slams
        
        # Matrix defaults
        self.data["matrix"] = {
            "include": [{"repeats": 1}]
        }
        self.data["output"] = {"root_dir": "results/runs"}
        
        # Save
        filename = f"{self.data['name'].lower().replace(' ', '_')}.yaml"
        out_path = Path("configs/matrices") / filename
        out_path.parent.mkdir(parents=True, exist_ok=True)
        
        try:
            with open(out_path, 'w') as f:
                yaml.dump(self.data, f, sort_keys=False)
            
            QMessageBox.information(self, "Success", f"Configuration created: {filename}")
            self.config_created.emit()
            self.accept()
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Failed to save: {e}")

    def run_import(self):
        src = self.imp_path.text().strip()
        if not src or not Path(src).exists():
             QMessageBox.warning(self, "Error", "Invalid file path.")
             return
             
        dest = Path("configs/matrices") / Path(src).name
        try:
            shutil.copy(src, dest)
            QMessageBox.information(self, "Success", "Configuration imported.")
            self.config_created.emit()
            self.accept()
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Import failed: {e}")

from PyQt5.QtWidgets import QWidget
