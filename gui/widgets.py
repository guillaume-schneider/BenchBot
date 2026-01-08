"""Reusable PyQt5 widgets for the GUI.

Provides custom widgets with modern styling and animations.
"""
from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton, QFrame, 
    QProgressBar, QGraphicsDropShadowEffect, QGridLayout
)
from PyQt5.QtCore import Qt, pyqtSignal
from PyQt5.QtGui import QColor, QFont, QCursor
import collections

class ConfigCard(QFrame):
    """Card widget representing a benchmark configuration.
    
    Displays configuration metadata, status, progress, and provides
    quick actions (Run/Stop/Edit).
    
    Signals:
        run_clicked: Emitted when Run button is clicked
        stop_clicked: Emitted when Stop button is clicked
        card_clicked: Emitted when card is clicked
        edit_clicked: Emitted when Edit button is clicked
    
    Args:
        path: Path to the configuration file
        data: Configuration data dictionary
        parent: Optional parent widget
    """
    """
    A card widget representing a benchmark configuration.
    Displays status, progress, and quick actions (Run/Stop).
    """
    run_clicked = pyqtSignal()
    stop_clicked = pyqtSignal()
    card_clicked = pyqtSignal()
    edit_clicked = pyqtSignal()

    def __init__(self, path, data, parent=None):
        super().__init__(parent)
        self.path = path
        self.data = data
        self.is_running = False
        
        self.setObjectName("configCard")
        self.setProperty("class", "card")
        self.setStyleSheet("""
            QFrame {
                background-color: rgba(30, 41, 59, 0.7);
                border: 1px solid #334155;
                border-radius: 12px;
            }
            QFrame:hover {
                background-color: rgba(30, 41, 59, 0.9);
                border: 1px solid #6366f1;
            }
        """)
        self.setCursor(QCursor(Qt.PointingHandCursor))
        
        self.layout = QVBoxLayout(self)
        self.layout.setContentsMargins(20, 20, 20, 20)
        self.layout.setSpacing(15)
        
        # Header
        header = QHBoxLayout()
        name_lbl = QLabel(data.get("name", "Unnamed"))
        name_lbl.setStyleSheet("font-size: 18px; font-weight: bold; color: #f8fafc; border: none; background: transparent;")
        header.addWidget(name_lbl)
        header.addStretch()
        
        self.status_badge = QLabel("IDLE")
        self.status_badge.setStyleSheet("""
            background-color: #334155; color: #94a3b8; padding: 4px 8px; border-radius: 4px; font-size: 12px; font-weight: bold;
        """)
        header.addWidget(self.status_badge)
        self.layout.addLayout(header)
        
        # Info Grid
        info = QGridLayout()
        info.setHorizontalSpacing(20)
        info.setVerticalSpacing(8)
        
        datasets = len(data.get("datasets", []))
        slams = len(data.get("slams", []))
        
        # Calculate total jobs
        total_jobs = 0
        for inc in data.get("matrix", {}).get("include", []):
             seeds = len(inc.get("seeds", [0]))
             slams_cnt = len(inc.get("slams", []))
             repeats = inc.get("repeats", 1)
             total_jobs += seeds * slams_cnt * repeats
             
        def add_info(label, value, row, col):
            l = QLabel(label)
            l.setStyleSheet("color: #94a3b8; font-size: 14px; border: none; background: transparent;")
            v = QLabel(str(value))
            v.setStyleSheet("color: #f8fafc; font-weight: 600; font-size: 14px; border: none; background: transparent;")
            info.addWidget(l, row, col)
            info.addWidget(v, row, col+1)

        add_info("Datasets", datasets, 0, 0)
        add_info("Algorithms", slams, 0, 2)
        add_info("Total Jobs", total_jobs, 1, 0)
        
        self.layout.addLayout(info)
        
        # Progress Bar
        self.pbar = QProgressBar()
        self.pbar.setTextVisible(False)
        self.pbar.setFixedHeight(6)
        self.pbar.setStyleSheet("""
            QProgressBar { border: none; background-color: #1e293b; border-radius: 3px; }
            QProgressBar::chunk { background-color: #6366f1; border-radius: 3px; }
        """)
        self.pbar.hide()
        self.layout.addWidget(self.pbar)
        
        self.metrics_row = QHBoxLayout()
        self.cpu_lbl = QLabel("CPU: 0%")
        self.cpu_lbl.setStyleSheet("color: #818cf8; font-size: 11px; font-weight: bold;")
        self.ram_lbl = QLabel("RAM: 0MB")
        self.ram_lbl.setStyleSheet("color: #10b981; font-size: 11px; font-weight: bold;")
        self.metrics_row.addWidget(self.cpu_lbl)
        self.metrics_row.addWidget(self.ram_lbl)
        self.metrics_row.addStretch()
        
        self.metrics_container = QWidget()
        self.metrics_container.setLayout(self.metrics_row)
        self.metrics_container.hide()
        self.layout.addWidget(self.metrics_container)

        self.layout.addStretch()
        
        # Actions
        actions = QHBoxLayout()
        
        # Edit Button
        self.edit_btn = QPushButton("Edit")
        self.edit_btn.setFixedSize(80, 32)
        self.edit_btn.setStyleSheet("""
            QPushButton { background-color: #334155; color: #f8fafc; border: none; border-radius: 6px; font-weight: bold; }
            QPushButton:hover { background-color: #475569; }
        """)
        self.edit_btn.clicked.connect(self.on_edit)
        actions.addWidget(self.edit_btn)
        
        actions.addStretch()
        
        self.run_btn = QPushButton("RUN")
        self.run_btn.setFixedSize(80, 32)
        self.run_btn.setStyleSheet("""
            QPushButton { background-color: #6366f1; color: white; border: none; border-radius: 6px; font-weight: bold; }
            QPushButton:hover { background-color: #4f46e5; }
        """)
        self.run_btn.clicked.connect(self.on_run)
        actions.addWidget(self.run_btn)
        
        self.stop_btn = QPushButton("STOP")
        self.stop_btn.setFixedSize(80, 32)
        self.stop_btn.setStyleSheet("""
            QPushButton { background-color: rgba(239, 68, 68, 0.2); color: #ef4444; border: 1px solid #ef4444; border-radius: 6px; font-weight: bold; }
            QPushButton:hover { background-color: rgba(239, 68, 68, 0.3); }
        """)
        self.stop_btn.clicked.connect(self.on_stop)
        self.stop_btn.hide()
        actions.addWidget(self.stop_btn)
        
        self.layout.addLayout(actions)

    def mousePressEvent(self, event):
        self.card_clicked.emit()
        super().mousePressEvent(event)

    def on_run(self):
        self.run_clicked.emit()
        
    def on_stop(self):
        self.stop_clicked.emit()
        
    def on_edit(self):
        self.edit_clicked.emit()

    def set_running(self, running):
        self.is_running = running
        if running:
            self.status_badge.setText("RUNNING")
            self.status_badge.setStyleSheet("""
                background-color: rgba(99, 102, 241, 0.2); color: #818cf8; 
                padding: 4px 8px; border-radius: 4px; font-size: 12px; font-weight: bold;
            """)
            self.run_btn.hide()
            self.stop_btn.show()
            self.pbar.show()
            self.metrics_container.show()
        else:
            self.status_badge.setText("IDLE")
            self.status_badge.setStyleSheet("""
                background-color: #334155; color: #94a3b8; 
                padding: 4px 8px; border-radius: 4px; font-size: 12px; font-weight: bold;
            """)
            self.run_btn.show()
            self.stop_btn.hide()
            self.pbar.hide()
            self.metrics_container.hide()

    def update_live_metrics(self, cpu, ram):
        self.cpu_lbl.setText(f"CPU: {cpu}%")
        self.ram_lbl.setText(f"RAM: {int(ram)}MB")

    def update_progress(self, current, total):
        if total > 0:
            self.pbar.setValue(int((current / total) * 100))
