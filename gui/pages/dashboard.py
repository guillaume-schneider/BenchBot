from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QGridLayout, QLabel, QScrollArea, QPushButton, QHBoxLayout
)
from PyQt5.QtCore import Qt, pyqtSignal
from pathlib import Path
import yaml
from gui.widgets import ConfigCard
from gui.wizard import ConfigWizard

class DashboardPage(QWidget):
    config_selected = pyqtSignal(str, dict)  # Path, Data
    run_requested = pyqtSignal(str, dict)    # Path, Data
    stop_requested = pyqtSignal(str)         # Path (no data needed for stop)
    edit_requested = pyqtSignal(str)
    
    def __init__(self, parent=None):
        super().__init__(parent)
        self.layout = QVBoxLayout(self)
        self.layout.setContentsMargins(40, 40, 40, 40)
        self.cards = {} # Path -> Widget
        
        self.init_ui()
        self.refresh_configs()

    def init_ui(self):
        # Header
        header_row = QHBoxLayout()
        title = QLabel("Dashboard")
        title.setObjectName("headerLabel")
        header_row.addWidget(title)
        
        header_row.addStretch()
        
        self.create_btn = QPushButton("Create New")
        self.create_btn.setFixedSize(120, 36)
        self.create_btn.setStyleSheet("""
            QPushButton {
                background-color: #2563eb; color: #ffffff; border: none; border-radius: 8px; font-weight: 600;
            }
            QPushButton:hover { background-color: #1d4ed8; }
        """)
        self.create_btn.clicked.connect(self.open_wizard)
        header_row.addWidget(self.create_btn)
        
        self.refresh_btn = QPushButton("Refresh")
        self.refresh_btn.setFixedSize(100, 36)
        self.refresh_btn.setStyleSheet("""
            QPushButton {
                background-color: #334155; color: #f8fafc; border: 1px solid #475569; border-radius: 8px; font-weight: 600;
            }
            QPushButton:hover { background-color: #475569; }
        """)
        self.refresh_btn.clicked.connect(self.refresh_configs)
        header_row.addWidget(self.refresh_btn)
        
        self.layout.addLayout(header_row)
        
        # Grid Area
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setStyleSheet("background: transparent; border: none;")
        
        self.grid_container = QWidget()
        self.grid_layout = QGridLayout(self.grid_container)
        self.grid_layout.setSpacing(20)
        self.grid_layout.setAlignment(Qt.AlignTop | Qt.AlignLeft)
        
        scroll.setWidget(self.grid_container)
        self.layout.addWidget(scroll)

    def open_wizard(self):
        wiz = ConfigWizard(self)
        wiz.config_created.connect(self.refresh_configs)
        wiz.exec_()

    def refresh_configs(self, running_config=None):
        # Clear existing
        for i in reversed(range(self.grid_layout.count())): 
            self.grid_layout.itemAt(i).widget().setParent(None)
        self.cards = {}
            
        matrices_dir = Path("configs/matrices")
        if not matrices_dir.exists():
            return
            
        row, col = 0, 0
        cols_per_row = 3
        
        for yaml_file in sorted(matrices_dir.glob("*.yaml")):
            try:
                from runner.resolve import load_yaml
                
                data = load_yaml(yaml_file)
                    
                card = ConfigCard(str(yaml_file), data)
                card.card_clicked.connect(lambda p=str(yaml_file), d=data: self.config_selected.emit(p, d))
                card.run_clicked.connect(lambda p=str(yaml_file), d=data: self.run_requested.emit(p, d))
                card.stop_clicked.connect(lambda p=str(yaml_file): self.stop_requested.emit(p))
                card.edit_clicked.connect(lambda p=str(yaml_file): self.edit_requested.emit(p))
                
                # Restore running state
                if running_config and str(yaml_file) == running_config:
                    card.set_running(True)
                
                self.grid_layout.addWidget(card, row, col)
                self.cards[str(yaml_file)] = card
                
                col += 1
                if col >= cols_per_row:
                    col = 0
                    row += 1
            except Exception as e:
                print(f"Error loading {yaml_file}: {e}")
