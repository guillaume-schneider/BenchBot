from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, QFrame, QFormLayout, 
    QLineEdit, QPushButton, QCheckBox, QMessageBox, QFileDialog
)
from pathlib import Path
 # Fix imports based on new location
from gt_map import generate_map, show_map

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
        self.sdf_path = QLineEdit("worlds/model.sdf")
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
        self.out_name = QLineEdit("generated_maps/map_gt")
        
        form_layout.addRow("Resolution (m/px):", self.res)
        form_layout.addRow("Laser Height (m):", self.laser_z)
        form_layout.addRow("Padding (m):", self.padding)
        form_layout.addRow("Output Base Name:", self.out_name)

        # Options
        self.gen_png = QCheckBox("Generate PNG image")
        self.gen_png.setChecked(True)
        self.gen_png.setStyleSheet("""
            QCheckBox { color: #f8fafc; }
            QCheckBox::indicator { width: 18px; height: 18px; }
            QCheckBox::indicator:unchecked { background-color: #1e293b; border: 1px solid #475569; border-radius: 3px; }
            QCheckBox::indicator:checked { background-color: #6366f1; border: 1px solid #6366f1; border-radius: 3px; image: url(none); }
        """)
        self.gen_debug = QCheckBox("Generate Debug geometry plot")
        self.gen_debug.setChecked(True)
        self.gen_debug.setStyleSheet("""
            QCheckBox { color: #f8fafc; }
            QCheckBox::indicator { width: 18px; height: 18px; }
            QCheckBox::indicator:unchecked { background-color: #1e293b; border: 1px solid #475569; border-radius: 3px; }
            QCheckBox::indicator:checked { background-color: #6366f1; border: 1px solid #6366f1; border-radius: 3px; image: url(none); }
        """)
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
