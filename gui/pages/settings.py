from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton, 
    QFrame, QMessageBox, QComboBox, QCheckBox, QScrollArea
)
from PyQt5.QtCore import Qt, QSettings
from PyQt5.QtGui import QIcon

class SettingsPage(QWidget):
    def __init__(self, main_window=None, parent=None):
        super().__init__(parent)
        self.main_window = main_window
        self.layout = QVBoxLayout(self)
        self.layout.setContentsMargins(30, 30, 30, 30)
        
        self.settings = QSettings("SlamBench", "Orchestrator")
        
        self.init_ui()
        
    def init_ui(self):
        # Header
        header = QLabel("Settings")
        header.setStyleSheet("font-size: 24px; font-weight: bold; color: #f8fafc; margin-bottom: 20px;")
        self.layout.addWidget(header)
        
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QFrame.NoFrame)
        scroll.setStyleSheet("background: transparent;")
        
        content = QWidget()
        content_layout = QVBoxLayout(content)
        content_layout.setSpacing(20)
        
        # --- Appearance ---
        app_card = self._create_card("Appearance")
        app_layout = QVBoxLayout(app_card)
        
        # Theme Toggle
        theme_row = QHBoxLayout()
        theme_label = QLabel("Theme")
        theme_label.setStyleSheet("color: #e2e8f0; font-size: 14px;")
        
        self.theme_combo = QComboBox()
        self.theme_combo.addItems(["Dark (Default)", "Light"])
        
        # Load saved theme
        saved_theme = self.settings.value("theme", "Dark (Default)")
        self.theme_combo.setCurrentText(saved_theme)
        
        self.theme_combo.currentTextChanged.connect(self.on_theme_changed)
        
        theme_row.addWidget(theme_label)
        theme_row.addStretch()
        theme_row.addWidget(self.theme_combo)
        app_layout.addLayout(theme_row)
        
        content_layout.addWidget(app_card)
        
        # --- Simulator Management ---
        # We reuse the SimulatorManagementPage logic but embed it here
        # Or better: we create a wrapper around it
        from gui.pages.tools import SimulatorManagementPage
        
        sim_card = self._create_card("Simulators")
        sim_layout = QVBoxLayout(sim_card)
        
        # Instantiate the page but strip margins because it's inside a card
        self.sim_mgr_widget = SimulatorManagementPage()
        self.sim_mgr_widget.layout.setContentsMargins(0, 0, 0, 0)
        
        # Hide the internal header of SimulatorManagementPage since we have the card title
        for child in self.sim_mgr_widget.children():
            if isinstance(child, QLabel) and child.objectName() == "headerLabel":
                child.hide()
        
        sim_layout.addWidget(self.sim_mgr_widget)
        content_layout.addWidget(sim_card)
        
        # --- Execution / Docker ---
        exec_card = self._create_card("Execution")
        exec_layout = QVBoxLayout(exec_card)
        
        docker_row = QHBoxLayout()
        docker_label = QLabel("Run in Docker (Experimental)")
        docker_label.setStyleSheet("color: #e2e8f0; font-size: 14px;")
        
        self.docker_cb = QCheckBox()
        self.docker_cb.setChecked(self.settings.value("run_in_docker", "false") == "true")
        self.docker_cb.toggled.connect(self.on_docker_toggled)
        
        docker_row.addWidget(docker_label)
        docker_row.addStretch()
        docker_row.addWidget(self.docker_cb)
        exec_layout.addLayout(docker_row)
        
        self.build_btn = QPushButton("Build Docker Image")
        self.build_btn.setStyleSheet("background-color: #334155; color: white; padding: 5px; border-radius: 4px; font-size: 11px;")
        self.build_btn.clicked.connect(self.build_docker_image)
        exec_layout.addWidget(self.build_btn)

        docker_info = QLabel("Isolation & Portability. Requires Docker installed.")
        docker_info.setStyleSheet("color: #94a3b8; font-size: 11px; font-style: italic;")
        exec_layout.addWidget(docker_info)
        
        content_layout.addWidget(exec_card)
        
        content_layout.addStretch()
        scroll.setWidget(content)
        self.layout.addWidget(scroll)
        
        # Apply initial theme
        self.on_theme_changed(saved_theme)

    def _create_card(self, title):
        card = QFrame()
        card.setObjectName("settingsCard")
        card.setStyleSheet("""
            #settingsCard {
                background-color: #1e293b;
                border: 1px solid #334155;
                border-radius: 8px;
            }
        """)
        
        # Add title
        l = QVBoxLayout()
        t = QLabel(title)
        t.setStyleSheet("font-size: 16px; font-weight: bold; color: #f1f5f9; padding: 10px; border-bottom: 1px solid #334155;")
        l.addWidget(t)
        
        # Container for content
        # We set the card's layout to this wrapper
        # wait, standard widgets have one layout.
        # So we return the card, caller sets layout. No.
        # We return the card widget.
        return card

    def on_theme_changed(self, text):
        self.settings.setValue("theme", text)
        is_dark = "Dark" in text
        
        if self.main_window:
            self.apply_theme(is_dark)
            
    def on_docker_toggled(self, checked):
        self.settings.setValue("run_in_docker", "true" if checked else "false")
        QMessageBox.information(self, "Docker Execution", 
            "Docker execution mode " + ("ENABLED" if checked else "DISABLED") + 
            ".\nNote: Benchmarks will now use 'docker-compose' for isolation.")
            
    def build_docker_image(self):
        # We can't easily show streaming output in a QMessageBox, but we can launch it
        # and tell the user it started.
        reply = QMessageBox.question(self, "Build Docker", 
            "This will build the 'slam-bench-orchestrator:latest' image.\nIt may take several minutes. Continue?",
            QMessageBox.Yes | QMessageBox.No)
            
        if reply == QMessageBox.Yes:
            import subprocess
            try:
                # We could use a thread but for now just a simple blocking-ish start
                # Better: print to logs if we had a global log area
                self.build_btn.setEnabled(False)
                self.build_btn.setText("Building (check terminal)...")
                # Non-blocking-ish
                cmd = ["docker", "build", "-t", "slam-bench-orchestrator:latest", "."]
                subprocess.Popen(cmd, cwd=str(self.main_window.PROJECT_ROOT if hasattr(self.main_window, 'PROJECT_ROOT') else "."))
                QMessageBox.information(self, "Build Started", "Docker build started in background.\nPlease check your terminal for progress.")
            except Exception as e:
                QMessageBox.critical(self, "Error", f"Could not start docker build: {e}")
            
    def apply_theme(self, is_dark):
        # We define stylesheets for both modes
        if is_dark:
            from gui.utils import STYLE_SHEET # Default dark
            self.main_window.setStyleSheet(STYLE_SHEET)
            # Update local styles specific to this page if needed
            self.theme_combo.setStyleSheet("")
        else:
            # Simple Light Theme
            light_style = """
            QMainWindow, QWidget#mainScreen { background-color: #f1f5f9; color: #0f172a; }
            QWidget#sidebar { background-color: #ffffff; border-right: 1px solid #cbd5e1; }
            QLabel { color: #0f172a; }
            QPushButton#navButton {
                text-align: left;
                padding: 12px 30px;
                border: none;
                background-color: transparent;
                color: #475569;
                font-size: 14px;
                font-weight: 500;
                margin: 4px 10px;
                border-radius: 6px;
            }
            QPushButton#navButton:checked {
                background-color: #e2e8f0;
                color: #2563eb;
                font-weight: 600;
            }
            QPushButton#navButton:hover {
                background-color: #f8fafc;
            }
            QFrame[class="card"] {
                background-color: #ffffff;
                border: 1px solid #e2e8f0;
                border-radius: 12px;
            }
            QPushButton#actionButton {
                background-color: #2563eb;
                color: white;
                border: none;
                padding: 10px 20px;
                border-radius: 6px;
                font-weight: 600;
            }
            QPushButton#actionButton:hover { background-color: #1d4ed8; }
            QLineEdit {
                padding: 10px;
                background-color: #ffffff;
                border: 1px solid #cbd5e1;
                border-radius: 6px;
                color: #0f172a;
            }
            QTextEdit {
                background-color: #ffffff;
                color: #0f172a;
                border: 1px solid #cbd5e1;
            }
            QTableWidget {
                background-color: #ffffff;
                alternate-background-color: #f8fafc;
                color: #0f172a;
                gridline-color: #e2e8f0;
            }
            QHeaderView::section {
                background-color: #f1f5f9;
                color: #475569;
                border: none;
            }
            /* Override Settings Card for Light */
            #settingsCard {
                background-color: #ffffff;
                border: 1px solid #cbd5e1;
            }
            QLabel { color: #0f172a; } 
            """
            self.main_window.setStyleSheet(light_style)
