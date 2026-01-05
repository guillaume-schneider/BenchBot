from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, QSlider, QDoubleSpinBox, 
    QFrame, QGroupBox, QGridLayout, QPushButton, QCheckBox
)
from PyQt5.QtCore import Qt
import json
from pathlib import Path

class RobotManagerPage(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.settings_path = Path("configs/robot_settings.json")
        self.init_ui()
        self.load_settings()

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

        # Save Button
        save_btn = QPushButton("Save & Apply Settings")
        save_btn.setStyleSheet("""
            QPushButton { background-color: #6366f1; color: white; font-weight: bold; border-radius: 6px; padding: 12px; font-size: 14px; }
            QPushButton:hover { background-color: #4f46e5; }
        """)
        save_btn.clicked.connect(self.save_settings)
        layout.addWidget(save_btn)

        layout.addStretch()

    def apply_preset(self, v):
        self.range_spin.setValue(v["range"])
        self.noise_spin.setValue(v["noise"])
        self.speed_slider.setValue(v["speed"])

    def load_settings(self):
        if self.settings_path.exists():
            try:
                with open(self.settings_path) as f:
                    data = json.load(f)
                    self.enable_cb.setChecked(data.get("enabled", False))
                    self.range_spin.setValue(data.get("max_range", 10.0))
                    self.noise_spin.setValue(data.get("noise_std", 0.0))
                    self.speed_slider.setValue(int(data.get("speed_scale", 1.0) * 100))
            except:
                pass

    def save_settings(self):
        data = {
            "enabled": self.enable_cb.isChecked(),
            "max_range": self.range_spin.value(),
            "noise_std": self.noise_spin.value(),
            "speed_scale": self.speed_slider.value() / 100.0
        }
        self.settings_path.parent.mkdir(parents=True, exist_ok=True)
        with open(self.settings_path, "w") as f:
            json.dump(data, f, indent=4)
