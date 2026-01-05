from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton, QTabWidget, 
    QTextEdit, QGroupBox, QComboBox, QSplitter, QFrame, QFormLayout,
    QScrollArea, QLineEdit, QCheckBox, QMessageBox, QFileDialog
)
from PyQt5.QtCore import Qt, pyqtSignal
from PyQt5.QtGui import QFont
import yaml
from gui.utils import YamlHighlighter

class VisualEditorWidget(QScrollArea):
    def __init__(self):
        super().__init__()
        self.setWidgetResizable(True)
        self.container = QWidget()
        self.layout = QVBoxLayout(self.container)
        self.setWidget(self.container)
        self.setStyleSheet("background-color: transparent; border: none;")

        self.init_ui()

    def init_ui(self):
        self.layout.setSpacing(20)
        self.layout.setContentsMargins(15, 15, 25, 15)

        # General Section
        self.general_group = self.create_section("General")
        gen_layout = QFormLayout(self.general_group)
        self.name_edit = QLineEdit()
        gen_layout.addRow("Name:", self.name_edit)
        self.layout.addWidget(self.general_group)

        # Workspace Section
        self.ws_group = self.create_section("Workspace")
        ws_layout = QFormLayout(self.ws_group)
        self.ros_distro = QLineEdit()
        self.use_sim_time = QCheckBox("Use Sim Time")
        ws_layout.addRow("ROS Distro:", self.ros_distro)
        ws_layout.addRow(self.use_sim_time)
        self.layout.addWidget(self.ws_group)

        # Defaults Section
        self.defaults_group = self.create_section("Defaults")
        def_layout = QFormLayout(self.defaults_group)
        self.warmup_edit = QLineEdit()
        self.timeout_edit = QLineEdit()
        self.rosbag_topics = QLineEdit()
        def_layout.addRow("Warmup (s):", self.warmup_edit)
        def_layout.addRow("Timeout (s):", self.timeout_edit)
        def_layout.addRow("Rosbag Topics:", self.rosbag_topics)
        self.layout.addWidget(self.defaults_group)
        
        # Datasets
        self.datasets_group = self.create_section("Datasets")
        self.datasets_layout = QVBoxLayout(self.datasets_group)
        add_ds_btn = QPushButton("+ Add Dataset")
        add_ds_btn.setFixedWidth(150)
        add_ds_btn.setObjectName("actionButton")
        add_ds_btn.clicked.connect(lambda: self.add_dataset_item({}))
        self.datasets_layout.addWidget(add_ds_btn)
        self.layout.addWidget(self.datasets_group)

        # SLAMs
        self.slams_group = self.create_section("SLAM Profiles")
        self.slams_layout = QVBoxLayout(self.slams_group)
        add_slam_btn = QPushButton("+ Add SLAM")
        add_slam_btn.setFixedWidth(150)
        add_slam_btn.setObjectName("actionButton")
        add_slam_btn.clicked.connect(lambda: self.add_slam_item({}))
        self.slams_layout.addWidget(add_slam_btn)
        self.layout.addWidget(self.slams_group)

        # Matrix
        self.matrix_group = self.create_section("Execution Matrix (Include)")
        self.matrix_layout = QVBoxLayout(self.matrix_group)
        add_inc_btn = QPushButton("+ Add Entry")
        add_inc_btn.setFixedWidth(150)
        add_inc_btn.setObjectName("actionButton")
        add_inc_btn.clicked.connect(lambda: self.add_include_item({}))
        self.matrix_layout.addWidget(add_inc_btn)
        self.layout.addWidget(self.matrix_group)

        self.layout.addStretch()
        
    def create_section(self, title):
        group = QGroupBox(title)
        group.setStyleSheet("""
            QGroupBox { 
                background-color: rgba(30, 41, 59, 0.4); 
                border: 1px solid #334155; 
                border-radius: 8px; 
                margin-top: 10px; 
                font-weight: bold; 
                color: #f8fafc;
            } 
            QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0 5px; }
        """)
        return group

    def create_card(self):
        f = QFrame()
        f.setProperty("class", "card") # Inherit global style
        f.setStyleSheet("background-color: rgba(15, 23, 42, 0.6); margin-bottom: 10px; border: 1px dashed #475569;")
        return f

    def add_dataset_item(self, data):
        w = self.create_card()
        w.original_data = data # Preserve hidden fields like scenario
        l = QVBoxLayout(w)
        f = QFormLayout()
        id_edit = QLineEdit(data.get("id", ""))
        kind_edit = QLineEdit(data.get("kind", "sim_gazebo"))
        
        # Prefer world_model, fallback to old GT if present (for display only)
        wm_val = data.get("world_model", "")
        if not wm_val and "ground_truth" in data:
             # Legacy support or manual
             wm_val = data.get("ground_truth", {}).get("map_path", "")
             
        wm_edit = QLineEdit(wm_val)
        
        f.addRow("ID:", id_edit)
        f.addRow("Kind:", kind_edit)
        f.addRow("World Model (SDF):", wm_edit)
        l.addLayout(f)
        
        rem_btn = QPushButton("Remove")
        rem_btn.setObjectName("dangerButton")
        rem_btn.clicked.connect(lambda: w.deleteLater())
        l.addWidget(rem_btn)
        
        self.datasets_layout.insertWidget(self.datasets_layout.count()-1, w)

    def add_slam_item(self, data):
        w = self.create_card()
        w.original_data = data
        l = QVBoxLayout(w)
        f = QFormLayout()
        id_edit = QLineEdit(data.get("id", ""))
        profile_edit = QLineEdit(data.get("profile", ""))
        f.addRow("ID:", id_edit)
        f.addRow("Profile:", profile_edit)
        l.addLayout(f)
        rem_btn = QPushButton("Remove")
        rem_btn.setObjectName("dangerButton")
        rem_btn.clicked.connect(lambda: w.deleteLater())
        l.addWidget(rem_btn)
        self.slams_layout.insertWidget(self.slams_layout.count()-1, w)

    def add_include_item(self, data):
        w = self.create_card()
        w.original_data = data
        l = QVBoxLayout(w)
        f = QFormLayout()
        
        ds_edit = QLineEdit(data.get("dataset", ""))
        slams_edit = QLineEdit(", ".join(data.get("slams", [])))
        seeds = data.get("seeds", [0])
        seeds_edit = QLineEdit(", ".join(map(str, seeds)))
        repeats_edit = QLineEdit(str(data.get("repeats", 1)))
        
        f.addRow("Dataset ID:", ds_edit)
        f.addRow("SLAM IDs:", slams_edit)
        f.addRow("Seeds:", seeds_edit)
        f.addRow("Repeats:", repeats_edit)
        
        # Degradation
        deg = data.get("degradation", {})
        d_widget = QWidget()
        d_layout = QHBoxLayout(d_widget)
        d_layout.setContentsMargins(0,0,0,0)
        
        cb_en = QCheckBox("Active")
        cb_en.setChecked(deg.get("enabled", False))
        
        noise_ed = QLineEdit(str(deg.get("noise_std", 0.0)))
        noise_ed.setPlaceholderText("Noise Std")
        noise_ed.setFixedWidth(60)
        
        range_ed = QLineEdit(str(deg.get("max_range", 10.0)))
        range_ed.setPlaceholderText("Range")
        range_ed.setFixedWidth(60)
        
        speed_ed = QLineEdit(str(deg.get("speed_scale", 1.0)))
        speed_ed.setPlaceholderText("Speed")
        speed_ed.setFixedWidth(60)
        
        d_layout.addWidget(cb_en)
        d_layout.addWidget(QLabel("Noise:"))
        d_layout.addWidget(noise_ed)
        d_layout.addWidget(QLabel("Range:"))
        d_layout.addWidget(range_ed)
        d_layout.addWidget(QLabel("Speed:"))
        d_layout.addWidget(speed_ed)
        
        f.addRow("Degradation:", d_widget)
        
        l.addLayout(f)
        
        # Store refs
        w.fields = {
            "dataset": ds_edit, "slams": slams_edit, "seeds": seeds_edit, "repeats": repeats_edit,
            "deg_en": cb_en, "deg_noise": noise_ed, "deg_range": range_ed, "deg_speed": speed_ed
        }
        
        rem_btn = QPushButton("Remove")
        rem_btn.setObjectName("dangerButton")
        rem_btn.clicked.connect(lambda: w.deleteLater())
        l.addWidget(rem_btn)
        self.matrix_layout.insertWidget(self.matrix_layout.count()-1, w)

    def load_data(self, data):
        self._root_data = data # Store root data for top-level fields
        # ... (rest is handled in sub-calls)
        
        # Clear dynamic
        for layout in [self.datasets_layout, self.slams_layout, self.matrix_layout]:
            for i in reversed(range(layout.count()-1)): # Keep button
                w = layout.itemAt(i).widget()
                if w: w.deleteLater()
                
        self.name_edit.setText(str(data.get("name", "")))
        ws = data.get("workspace", {})
        self.ros_distro.setText(ws.get("ros_distro", "humble"))
        self.use_sim_time.setChecked(ws.get("use_sim_time", True))
        
        defs = data.get("defaults", {})
        self.warmup_edit.setText(str(defs.get("run", {}).get("warmup_s", 2.0)))
        self.timeout_edit.setText(str(defs.get("run", {}).get("timeout_s", 90.0)))
        self.rosbag_topics.setText(", ".join(defs.get("rosbag", {}).get("topics", [])))
        
        for d in data.get("datasets", []): self.add_dataset_item(d)
        for s in data.get("slams", []): self.add_slam_item(s)
        for i in data.get("matrix", {}).get("include", []): self.add_include_item(i)

    def get_data(self):
        # Start with a copy of loaded data to preserve top-level structure (like user comments, unknown fields?)
        # Actually yaml.safe_load loses comments anyway.
        # But we should try to preserve extra fields if we stored them.
        data = getattr(self, '_root_data', {}).copy()
        
        # Update managed fields
        data["name"] = self.name_edit.text()
        data["schema_version"] = 1
        
        ws = data.get("workspace", {})
        ws["ros_distro"] = self.ros_distro.text()
        ws["use_sim_time"] = self.use_sim_time.isChecked()
        data["workspace"] = ws
        
        defs = data.get("defaults", {})
        if "run" not in defs: defs["run"] = {}
        defs["run"]["warmup_s"] = float(self.warmup_edit.text() or 0)
        defs["run"]["timeout_s"] = float(self.timeout_edit.text() or 0)
        
        if "rosbag" not in defs: defs["rosbag"] = {}
        defs["rosbag"]["topics"] = [x.strip() for x in self.rosbag_topics.text().split(",") if x.strip()]
        data["defaults"] = defs
        
        data["datasets"] = []
        data["slams"] = []
        if "matrix" not in data: data["matrix"] = {}
        data["matrix"]["include"] = []
        
        # Helper to get form items
        def get_form_field(w, row):
            l = w.layout().itemAt(0) # FormLayout
            return l.itemAt(row, QFormLayout.FieldRole).widget().text()

        # Datasets
        for i in range(self.datasets_layout.count()-1):
            w = self.datasets_layout.itemAt(i).widget()
            # Merge with original
            ds = w.original_data.copy() if hasattr(w, 'original_data') else {}
            
            id_ = get_form_field(w, 0)
            kind = get_form_field(w, 1)
            wm = get_form_field(w, 2)
            
            ds["id"] = id_
            ds["kind"] = kind
            if wm.strip(): 
                ds["world_model"] = wm.strip()
            # Note: ground_truth is NOT removed explicitly, but if we don't set it, it stays if present in original
            # If user wants to remove GT, they can't via UI except by editing Raw.
            # But Auto-GT logic overrides it anyway.
            
            data["datasets"].append(ds)

        # SLAMs
        for i in range(self.slams_layout.count()-1):
            w = self.slams_layout.itemAt(i).widget()
            s = w.original_data.copy() if hasattr(w, 'original_data') else {}
            s["id"] = get_form_field(w, 0)
            s["profile"] = get_form_field(w, 1)
            data["slams"].append(s)
            
        # Matrix
        for i in range(self.matrix_layout.count()-1):
            w = self.matrix_layout.itemAt(i).widget()
            inc = w.original_data.copy() if hasattr(w, 'original_data') else {}
            
            if hasattr(w, 'fields'):
                inc["dataset"] = w.fields["dataset"].text()
                slams = [x.strip() for x in w.fields["slams"].text().split(",") if x.strip()]
                seeds = [int(x) for x in w.fields["seeds"].text().split(",") if x.strip().isdigit()]
                inc["repeats"] = int(w.fields["repeats"].text() or 1)
                inc["slams"] = slams
                inc["seeds"] = seeds if seeds else [0]
                
                # Degradation
                d = inc.get("degradation", {})
                d["enabled"] = w.fields["deg_en"].isChecked()
                d["noise_std"] = float(w.fields["deg_noise"].text() or 0)
                d["max_range"] = float(w.fields["deg_range"].text() or 10)
                d["speed_scale"] = float(w.fields["deg_speed"].text() or 1)
                inc["degradation"] = d
            else:
                # Fallback should not happen with new items
                continue
            
            data["matrix"]["include"].append(inc)
            
        return data

class ConfigEditorPage(QWidget):
    back_clicked = pyqtSignal()
    save_clicked = pyqtSignal(str, dict) # Path, Data
    
    def __init__(self, parent=None):
        super().__init__(parent)
        self.config_path = ""
        self.init_ui()
        
    def init_ui(self):
        # Header
        top_bar = QFrame()
        top_bar.setStyleSheet("background-color: #1e293b; border-bottom: 1px solid #334155;")
        top = QHBoxLayout(top_bar)
        
        back_btn = QPushButton("Cancel")
        back_btn.setObjectName("cancelButton")
        back_btn.clicked.connect(self.back_clicked.emit)
        top.addWidget(back_btn)
        
        self.title_lbl = QLabel("Editor")
        self.title_lbl.setStyleSheet("font-weight: bold; font-size: 16px; margin-left: 10px;")
        top.addWidget(self.title_lbl)
        
        top.addStretch()
        
        save_btn = QPushButton("Save Config")
        save_btn.setObjectName("actionButton")
        save_btn.clicked.connect(self.save)
        top.addWidget(save_btn)
        
        # Main Layout
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.addWidget(top_bar)
        
        # Tabs
        self.tabs = QTabWidget()
        self.tabs.setStyleSheet("""
            QTabWidget::pane { border: none; }
            QTabBar::tab { background: transparent; color: #94a3b8; padding: 10px 20px; font-weight: 600; }
            QTabBar::tab:selected { color: #818cf8; border-bottom: 2px solid #818cf8; }
        """)
        
        # Visual Tab
        self.visual_editor = VisualEditorWidget()
        self.tabs.addTab(self.visual_editor, "Visual Editor")
        
        # Raw Tab
        self.raw_view = QTextEdit()
        self.raw_view.setFont(QFont("Monospace", 12))
        self.highlighter = YamlHighlighter(self.raw_view.document())
        self.tabs.addTab(self.raw_view, "Raw YAML")
        
        layout.addWidget(self.tabs)
        
        # Sync Logic
        self.tabs.currentChanged.connect(self.on_tab_change)
        
    def load_config(self, path):
        self.config_path = path
        self.title_lbl.setText(f"Editing: {path}")
        try:
            with open(path, 'r') as f:
                data = yaml.safe_load(f)
            self.visual_editor.load_data(data)
            self.raw_view.setPlainText(yaml.dump(data, sort_keys=False))
        except Exception as e:
            QMessageBox.critical(self, "Error", str(e))
            
    def on_tab_change(self, index):
        # Index 0 = Visual, 1 = Raw
        if index == 1:
            # Switch to Raw -> Update from Visual
            try:
                data = self.visual_editor.get_data()
                self.raw_view.setPlainText(yaml.dump(data, sort_keys=False))
            except Exception as e:
                print(f"Sync Error: {e}")
        else:
            # Switch to Visual -> Update from Raw
            try:
                data = yaml.safe_load(self.raw_view.toPlainText())
                self.visual_editor.load_data(data)
            except Exception as e:
                QMessageBox.warning(self, "Parse Error", f"Invalid YAML: {e}\nReverting to pre-switch state.")
                self.tabs.setCurrentIndex(1) # Stay on Raw
                
    def save(self):
        # Determine current data source
        if self.tabs.currentIndex() == 0:
             data = self.visual_editor.get_data()
        else:
             try:
                 data = yaml.safe_load(self.raw_view.toPlainText())
             except Exception as e:
                 QMessageBox.critical(self, "Error", f"Invalid YAML: {e}")
                 return
                 
        try:
            with open(self.config_path, 'w') as f:
                yaml.dump(data, f, sort_keys=False)
            QMessageBox.information(self, "Saved", "Configuration saved successfully.")
            self.save_clicked.emit(self.config_path, data)
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Failed to save: {e}")
