from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, QFrame, QGridLayout
)
from PyQt5.QtCore import Qt, pyqtSignal
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg
from matplotlib.figure import Figure
import matplotlib.pyplot as plt
import collections

class LiveMonitorPage(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.max_points = 60 # 1 minute at 1Hz
        self.cpu_data = collections.deque([0.0] * self.max_points, maxlen=self.max_points)
        self.ram_data = collections.deque([0.0] * self.max_points, maxlen=self.max_points)
        self.time_data = list(range(self.max_points))
        
        self.init_ui()

    def init_ui(self):
        self.layout = QVBoxLayout(self)
        self.layout.setContentsMargins(30, 30, 30, 30)
        self.layout.setSpacing(20)
        
        # Header
        header = QLabel("Live System Monitor")
        header.setStyleSheet("font-size: 24px; font-weight: bold; color: #f8fafc;")
        self.layout.addWidget(header)
        
        # Stats Row (Cards)
        stats_layout = QHBoxLayout()
        self.cpu_card = self._create_stat_card("Current CPU", "0.0 %", "#6366f1")
        self.ram_card = self._create_stat_card("Current RAM", "0.0 MB", "#10b981")
        stats_layout.addWidget(self.cpu_card)
        stats_layout.addWidget(self.ram_card)
        self.layout.addLayout(stats_layout)
        
        # Graphs
        graph_frame = QFrame()
        graph_frame.setStyleSheet("background-color: #1e293b; border-radius: 12px; border: 1px solid #334155;")
        graph_layout = QVBoxLayout(graph_frame)
        
        self.figure = Figure(figsize=(10, 6), facecolor='#1e293b')
        self.canvas = FigureCanvasQTAgg(self.figure)
        graph_layout.addWidget(self.canvas)
        
        self.ax_cpu = self.figure.add_subplot(211)
        self.ax_ram = self.figure.add_subplot(212)
        
        for ax in [self.ax_cpu, self.ax_ram]:
            ax.set_facecolor('#1e293b')
            ax.tick_params(colors='#94a3b8', labelsize=8)
            for spine in ax.spines.values():
                spine.set_color('#334155')
        
        self.cpu_line, = self.ax_cpu.plot(self.time_data, list(self.cpu_data), color='#6366f1', linewidth=2)
        self.ram_line, = self.ax_ram.plot(self.time_data, list(self.ram_data), color='#10b981', linewidth=2)
        
        self.ax_cpu.set_title("CPU Usage (%)", color='#f1f5f9', fontsize=10, loc='left')
        self.ax_ram.set_title("RAM Usage (MB)", color='#f1f5f9', fontsize=10, loc='left')
        
        self.figure.tight_layout(pad=3.0)
        self.layout.addWidget(graph_frame)
        
        # Info Footer
        self.info_lbl = QLabel("No active run.")
        self.info_lbl.setStyleSheet("color: #94a3b8; font-style: italic;")
        self.layout.addWidget(self.info_lbl)

    def _create_stat_card(self, title, value, color):
        card = QFrame()
        card.setStyleSheet(f"""
            QFrame {{
                background-color: #1e293b;
                border: 1px solid #334155;
                border-left: 4px solid {color};
                border-radius: 8px;
                padding: 15px;
            }}
        """)
        l = QVBoxLayout(card)
        t = QLabel(title)
        t.setStyleSheet("color: #94a3b8; font-size: 12px; font-weight: bold; border: none;")
        v = QLabel(value)
        v.setStyleSheet(f"color: {color}; font-size: 24px; font-weight: bold; border: none;")
        v.setObjectName("valueLabel")
        l.addWidget(t)
        l.addWidget(v)
        return card

    def update_metrics(self, data):
        cpu = data.get('cpu', 0.0)
        ram = data.get('ram', 0.0)
        
        # Update Cards
        self.cpu_card.findChild(QLabel, "valueLabel").setText(f"{cpu} %")
        self.ram_card.findChild(QLabel, "valueLabel").setText(f"{ram} MB")
        
        # Update Data
        self.cpu_data.append(cpu)
        self.ram_data.append(ram)
        
        # Update Plots
        self.cpu_line.set_ydata(list(self.cpu_data))
        self.ram_line.set_ydata(list(self.ram_data))
        
        # Rescale axes
        self.ax_cpu.relim()
        self.ax_cpu.autoscale_view()
        self.ax_ram.relim()
        self.ax_ram.autoscale_view()
        
        self.canvas.draw()

    def set_run_info(self, run_id):
        if run_id:
            self.info_lbl.setText(f"Monitoring active run: {run_id}")
            self.info_lbl.setStyleSheet("color: #60a5fa; font-weight: bold;")
        else:
            self.info_lbl.setText("No active run.")
            self.info_lbl.setStyleSheet("color: #94a3b8; font-style: italic;")
            # Clear data? 
            # self.cpu_data = collections.deque([0.0] * self.max_points, maxlen=self.max_points)
            # ...
