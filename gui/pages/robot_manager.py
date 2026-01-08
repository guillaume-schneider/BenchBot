"""Robot and sensor degradation manager page.

Allows users to simulate hardware limitations and sensor noise
for SLAM robustness testing.
"""

from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, QSlider, QDoubleSpinBox, 
    QFrame, QGroupBox, QGridLayout, QPushButton, QCheckBox, QApplication,
    QRadioButton, QButtonGroup, QFileDialog, QMessageBox
)
from PyQt5.QtCore import Qt
import json
import yaml
from pathlib import Path

class RobotManagerPage(QWidget):
    """Page for configuring robot hardware degradation parameters.
    
    Provides controls for:
    - LIDAR range and noise
    - Chassis speed scaling
    - Preset configurations
    - Saving to configuration files
    """


    def __init__(self, parent=None):
        super().__init__(parent)
        self.current_config_path = None
        self.init_ui()

    def init_ui(self):
        layout = QVBoxLayout(self)
        layout.setContentsMargins(30, 30, 30, 30)
        layout.setSpacing(20)

        # Header
        header = QLabel("Robot & Sensor Manager")
        header.setStyleSheet("font-size: 24px; font-weight: bold; color: #f8fafc;")
        layout.addWidget(header)

        # Description
        desc = QLabel("Simulate hardware limitations and sensor noise to test SLAM robustness.")
        desc.setStyleSheet("color: #94a3b8; font-style: italic;")

        layout.addWidget(desc)


        # Target Configuration
        file_group = QGroupBox("Target Configuration")
        file_group.setStyleSheet("QGroupBox { font-weight: bold; color: #cbd5e1; border: 1px solid #334155; padding: 10px; }")
        fg_layout = QHBoxLayout(file_group)
        
        self.lbl_path = QLabel("No configuration loaded. Please open a matrix.yaml file.")
        self.lbl_path.setStyleSheet("color: #ef4444; font-style: italic; font-weight: bold;")
        
        self.btn_load = QPushButton("Open Config File")
        self.btn_load.clicked.connect(self.browse_config)
        self.btn_load.setStyleSheet("background-color: #3b82f6; color: white; border: none; padding: 8px 16px; border-radius: 4px; font-weight: bold;")
        
        fg_layout.addWidget(self.lbl_path)
        fg_layout.addWidget(self.btn_load)
        
        layout.addWidget(file_group)

        # Main Switch
        self.enable_cb = QCheckBox("Enable Hardware Degradation")
        self.enable_cb.setStyleSheet("""
            QCheckBox { color: #f1f5f9; font-weight: bold; font-size: 16px; padding: 10px; }
            QCheckBox::indicator { width: 20px; height: 20px; }
        """)
        layout.addWidget(self.enable_cb)

        # Grid for Parameters
        content_layout = QGridLayout()
        
        # --- LIDAR Settings ---
        lidar_group = QGroupBox("LIDAR Sensor Emulation")
        lidar_group.setStyleSheet("QGroupBox { font-weight: bold; color: #6366f1; border: 1px solid #334155; margin-top: 15px; padding: 15px; }")
        lidar_layout = QGridLayout(lidar_group)

        # Range
        lidar_layout.addWidget(QLabel("Max Range (meters):"), 0, 0)
        self.range_spin = QDoubleSpinBox()
        self.range_spin.setRange(0.1, 30.0)
        self.range_spin.setValue(10.0)
        lidar_layout.addWidget(self.range_spin, 0, 1)

        # Noise
        lidar_layout.addWidget(QLabel("Gaussian Noise (std dev):"), 1, 0)
        self.noise_spin = QDoubleSpinBox()
        self.noise_spin.setRange(0.0, 1.0)
        self.noise_spin.setSingleStep(0.01)
        self.noise_spin.setValue(0.0)
        lidar_layout.addWidget(self.noise_spin, 1, 1)

        layout.addWidget(lidar_group)

        # --- Chassis Settings ---
        chassis_group = QGroupBox("Chassis & Actuators")
        chassis_group.setStyleSheet("QGroupBox { font-weight: bold; color: #10b981; border: 1px solid #334155; margin-top: 15px; padding: 15px; }")
        chassis_layout = QGridLayout(chassis_group)

        # Speed Scale
        chassis_layout.addWidget(QLabel("Speed Scaling (%):"), 0, 0)
        self.speed_slider = QSlider(Qt.Horizontal)
        self.speed_slider.setRange(10, 200)
        self.speed_slider.setValue(100)
        self.speed_label = QLabel("100%")
        self.speed_slider.valueChanged.connect(lambda v: self.speed_label.setText(f"{v}%"))
        chassis_layout.addWidget(self.speed_slider, 0, 1)
        chassis_layout.addWidget(self.speed_label, 0, 2)

        layout.addWidget(chassis_group)

        # Presets
        preset_layout = QHBoxLayout()
        for name, values in [
            ("Default (Clean)", {"range": 10.0, "noise": 0.0, "speed": 100}),
            ("Bad LIDAR (Short & Noisy)", {"range": 3.0, "noise": 0.05, "speed": 100}),
            ("Weak Motors (Slow)", {"range": 10.0, "noise": 0.0, "speed": 40}),
            ("Extreme (Stress Test)", {"range": 1.5, "noise": 0.15, "speed": 60}),
        ]:
            btn = QPushButton(name)
            btn.setStyleSheet("background-color: #334155; color: white; border: 1px solid #475569; padding: 8px; border-radius: 4px;")
            btn.clicked.connect(lambda checked, v=values: self.apply_preset(v))
            preset_layout.addWidget(btn)
        
        layout.addLayout(preset_layout)


        # Action Buttons Layout
        action_layout = QHBoxLayout()
        
        # Copy Button
        copy_btn = QPushButton("Copy Config (YAML)")
        copy_btn.setStyleSheet("""
            QPushButton { background-color: #334155; color: white; border: 1px solid #475569; padding: 12px; border-radius: 6px; font-weight: bold; }
            QPushButton:hover { background-color: #475569; }
        """)
        copy_btn.clicked.connect(self.copy_config)
        action_layout.addWidget(copy_btn)


        # Save Button
        save_btn = QPushButton("Save to Configuration File")
        save_btn.setStyleSheet("""
            QPushButton { background-color: #6366f1; color: white; font-weight: bold; border-radius: 6px; padding: 12px; font-size: 14px; }
            QPushButton:hover { background-color: #4f46e5; }
        """)
        save_btn.clicked.connect(self.save_settings)
        action_layout.addWidget(save_btn)
        
        layout.addLayout(action_layout)

        layout.addStretch()


    def copy_config(self):
        data = {
            "enabled": self.enable_cb.isChecked(),
            "max_range": self.range_spin.value(),
            "noise_std": self.noise_spin.value(),
            "speed_scale": self.speed_slider.value() / 100.0
        }
        # Format as YAML snippet for matrix
        yaml_str = f"""# Paste this into your matrix.yaml (under dataset or run)
degradation:
  enabled: {str(data['enabled']).lower()}
  max_range: {data['max_range']}
  noise_std: {data['noise_std']}
  speed_scale: {data['speed_scale']}
"""
        QApplication.clipboard().setText(yaml_str)
        
    def apply_preset(self, v):
        self.range_spin.setValue(v["range"])
        self.noise_spin.setValue(v["noise"])
        self.speed_slider.setValue(v["speed"])



    def browse_config(self):
        f, _ = QFileDialog.getOpenFileName(self, "Select Config", "configs/matrices", "YAML (*.yaml);;JSON (*.json)")
        if f:
             self.current_config_path = Path(f)
             self.lbl_path.setText(self.current_config_path.name)
             self.lbl_path.setStyleSheet("color: #22c55e; font-weight: bold;")
             self.load_settings()
             
    def load_settings(self):
        data = {}
        # Reset Defaults
        self.enable_cb.setChecked(False)
        self.range_spin.setValue(10.0)
        self.noise_spin.setValue(0.0)
        self.speed_slider.setValue(100)
        
        if not self.current_config_path or not self.current_config_path.exists():
            return
            
        path = self.current_config_path
        try:
            with open(path) as f:
                if path.suffix == '.json':
                        data = json.load(f)
                else:
                        full_data = yaml.safe_load(f) or {}
                        data = full_data.get("degradation", {})
                        if not data and "matrix" in full_data and "include" in full_data["matrix"]:
                            items = full_data["matrix"]["include"]
                            if items and isinstance(items, list):
                                data = items[0].get("degradation", {})
        except Exception as e:
            print(f"Error loading settings: {e}")

        if data:
             self.enable_cb.setChecked(data.get("enabled", False))
             self.range_spin.setValue(data.get("max_range", 10.0))
             self.noise_spin.setValue(data.get("noise_std", 0.0))
             self.speed_slider.setValue(int(data.get("speed_scale", 1.0) * 100))

    def save_settings(self):
        vals = {
            "enabled": self.enable_cb.isChecked(),
            "max_range": self.range_spin.value(),
            "noise_std": self.noise_spin.value(),
            "speed_scale": self.speed_slider.value() / 100.0
        }
        
        if not self.current_config_path:
             QMessageBox.warning(self, "No File", "Please load a configuration file first.")
             return

        # Edit YAML
        path = self.current_config_path
        try:
            full_data = {}
            if path.exists():
                with open(path) as f:
                    full_data = yaml.safe_load(f) or {}
            
            # Save logic (Matrix vs Root)
            if "matrix" in full_data and "include" in full_data["matrix"]:
                items = full_data["matrix"]["include"]
                if items and isinstance(items, list):
                    if "degradation" not in items[0]: items[0]["degradation"] = {}
                    items[0]["degradation"].update(vals)
            else:
                if "degradation" not in full_data: full_data["degradation"] = {}
                full_data["degradation"].update(vals)
                
            with open(path, "w") as f:
                yaml.dump(full_data, f, sort_keys=False)
            QMessageBox.information(self, "Saved", f"Updated config in {path.name}")
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Failed to save: {e}")
