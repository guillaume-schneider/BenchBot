import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from PyQt5.QtWidgets import QWidget, QVBoxLayout, QLabel, QHBoxLayout, QPushButton, QFrame, QCheckBox
from PyQt5.QtCore import pyqtSignal, QTimer, Qt
import pyqtgraph.opengl as gl
import pyqtgraph as pg
import threading
import math

class VisualizerNode(Node):
    """ROS 2 Node to bridge data to the 3D GUI"""
    scan_received = pyqtSignal(object)
    pose_received = pyqtSignal(object)

    def __init__(self, signals):
        super().__init__('gui_visualizer_node')
        self.signals = signals
        # Subscribe to multiple possible names for robustness
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(Odometry, 'odom', self.odom_callback, 10)

    def scan_callback(self, msg):
        # Convert LaserScan to 3D Points
        ranges = np.array(msg.ranges)
        angles = np.linspace(msg.angle_min, msg.angle_max, len(ranges))
        
        # Filter out inf/nan
        mask = np.isfinite(ranges)
        ranges = ranges[mask]
        angles = angles[mask]

        x = ranges * np.cos(angles)
        y = ranges * np.sin(angles)
        z = np.zeros_like(x)
        
        points = np.stack((x, y, z), axis=-1)
        self.signals.scan_received.emit(points)

    def odom_callback(self, msg):
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        self.signals.pose_received.emit({
            'x': pos.x, 'y': pos.y, 'z': pos.z,
            'qx': ori.x, 'qy': ori.y, 'qz': ori.z, 'qw': ori.w
        })

class VisualizerSignals(QWidget):
    """Simple wrapper to provide Qt Signals for the ROS Node"""
    scan_received = pyqtSignal(object)
    pose_received = pyqtSignal(object)

class VisualizerPage(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.layout = QVBoxLayout(self)
        self.layout.setContentsMargins(0, 0, 0, 0)
        
        # UI Elements
        self.init_ui()
        
        # ROS 2 Integration
        self.signals = VisualizerSignals()
        self.signals.scan_received.connect(self.update_scan)
        self.signals.pose_received.connect(self.update_pose)
        
        self.ros_node = None
        self.ros_thread = None
        self.active = False
        
        # Data
        self.traj_points = []
        self.latest_pose = None
        
        # Timer to check if we need to start ROS
        self.check_timer = QTimer()
        self.check_timer.timeout.connect(self.ensure_ros)
        self.check_timer.start(2000)

    def init_ui(self):
        # Header / Controls
        ctrl_frame = QFrame()
        ctrl_frame.setStyleSheet("background-color: #1e293b; border-bottom: 1px solid #334155;")
        ctrl_frame.setFixedHeight(60)
        ctrl_layout = QHBoxLayout(ctrl_frame)
        
        self.status_lbl = QLabel("3D Visualizer (Waiting for Odom/Scan...)")
        self.status_lbl.setStyleSheet("color: #94a3b8; font-weight: bold; margin-left: 20px;")
        ctrl_layout.addWidget(self.status_lbl)
        
        ctrl_layout.addStretch()
        
        reset_btn = QPushButton("Reset View")
        reset_btn.setStyleSheet("background-color: #334155; color: white; padding: 5px 15px; border-radius: 4px;")
        reset_btn.clicked.connect(self.reset_camera)
        ctrl_layout.addWidget(reset_btn)
        
        self.follow_cb = QCheckBox("Follow Robot")
        self.follow_cb.setStyleSheet("color: white; margin-right: 15px;")
        ctrl_layout.addWidget(self.follow_cb)
        
        clear_btn = QPushButton("Clear Trajectory")
        clear_btn.setStyleSheet("background-color: #ef4444; color: white; padding: 5px 15px; border-radius: 4px; margin-right: 20px;")
        clear_btn.clicked.connect(self.clear_data)
        ctrl_layout.addWidget(clear_btn)
        
        self.layout.addWidget(ctrl_frame)

        # 3. GL View
        self.view = gl.GLViewWidget()
        self.view.setBackgroundColor('#0f172a')
        self.view.setCameraPosition(distance=15, elevation=30, azimuth=45)
        self.layout.addWidget(self.view)
        
        # Grid
        self.grid = gl.GLGridItem()
        self.grid.setSize(20, 20)
        self.grid.setSpacing(1, 1)
        self.grid.setColor((51, 65, 85, 255))
        self.view.addItem(self.grid)
        
        # Lidar Points
        self.scan_item = gl.GLScatterPlotItem(pos=np.array([[0,0,0]]), color=(0.4, 0.5, 1.0, 1.0), size=2, pxMode=True)
        self.view.addItem(self.scan_item)
        
        # Trajectory
        self.traj_item = gl.GLLinePlotItem(pos=np.array([[0,0,0]]), color=(0.1, 0.8, 0.4, 1.0), width=2, antialias=True)
        self.view.addItem(self.traj_item)
        
        # Robot Representation (Composite)
        self.robot_axis = gl.GLAxisItem()
        self.robot_axis.setSize(0.5, 0.5, 0.5)
        self.view.addItem(self.robot_axis)
        
        # Add a more visible body for the robot
        self.robot_box = gl.GLBoxItem()
        self.robot_box.setSize(0.3, 0.3, 0.1)
        self.robot_box.translate(-0.15, -0.15, 0) # Center it
        self.robot_box.setParentItem(self.robot_axis)
        self.robot_box.setColor((59, 130, 246, 150)) # Blue-ish transparent

    def reset_camera(self):
        self.view.setCameraPosition(distance=15, elevation=30, azimuth=45)

    def clear_data(self):
        self.traj_points = []
        self.traj_item.setData(pos=np.array([[0,0,0]]))

    def ensure_ros(self):
        if not self.active:
            try:
                # ROS is now initialized in main.py
                self.ros_node = VisualizerNode(self.signals)
                self.ros_thread = threading.Thread(target=rclpy.spin, args=(self.ros_node,), daemon=True)
                self.ros_thread.start()
                self.active = True
                self.status_lbl.setText("3D Visualizer (ROS 2 Connected)")
                self.status_lbl.setStyleSheet("color: #10b981; font-weight: bold; margin-left: 20px;")
            except Exception as e:
                self.status_lbl.setText(f"3D Visualizer (ROS 2 Error: {str(e)[:30]})")
                self.status_lbl.setStyleSheet("color: #ef4444; font-weight: bold; margin-left: 20px;")

    def inject_pose(self, pose_data):
        """Fallback pose update from orchestrator metrics if ROS /odom is slow/missing"""
        if not pose_data: return
        yaw = pose_data.get('yaw', 0.0)
        # Convert yaw to quaternion
        self.update_pose({
            'x': pose_data.get('x', 0.0),
            'y': pose_data.get('y', 0.0),
            'z': 0.0,
            'qx': 0.0, 'qy': 0.0, 'qz': math.sin(yaw/2.0), 'qw': math.cos(yaw/2.0)
        })

    def update_scan(self, points):
        # Transform scan points from local robot frame to world frame (odom)
        if self.latest_pose:
            px, py, pz = self.latest_pose['x'], self.latest_pose['y'], self.latest_pose['z']
            qx, qy, qz, qw = self.latest_pose['qx'], self.latest_pose['qy'], self.latest_pose['qz'], self.latest_pose['qw']
            
            # 3D Rotation Matrix from Quaternion
            r00 = 1 - 2*(qy**2 + qz**2)
            r01 = 2*(qx*qy - qz*qw)
            r02 = 2*(qx*qz + qy*qw)
            r10 = 2*(qx*qy + qz*qw)
            r11 = 1 - 2*(qx**2 + qz**2)
            r12 = 2*(qy*qz - qx*qw)
            r20 = 2*(qx*qz - qy*qw)
            r21 = 2*(qy*qz + qx*qw)
            r22 = 1 - 2*(qx**2 + qy**2)
            
            R = np.array([[r00, r01, r02],
                          [r10, r11, r12],
                          [r20, r21, r22]])
            
            # Points are (N, 3). Transform: P_world = P_local * R^T + T
            world_points = points @ R.T + np.array([px, py, pz])
            self.scan_item.setData(pos=world_points)
        else:
            self.scan_item.setData(pos=points)

    def update_pose(self, pose):
        self.latest_pose = pose
        p = np.array([pose['x'], pose['y'], pose['z']])
        
        # Build 4x4 Transformation Matrix
        tr = pg.Transform3D()
        tr.translate(p[0], p[1], p[2])
        
        # Apply orientation (Quaternion to Axis-Angle)
        qw = max(-1.0, min(1.0, pose['qw']))
        angle = 2 * math.acos(qw)
        deg = math.degrees(angle)
        if deg > 0.1:
            s = math.sqrt(max(0, 1 - qw*qw))
            if s > 0.001:
                ax, ay, az = pose['qx']/s, pose['qy']/s, pose['qz']/s
                tr.rotate(deg, ax, ay, az)
        
        # Apply transform to the axis (and its child box)
        self.robot_axis.setTransform(tr)
        
        # Update Trajectory
        if not self.traj_points or np.linalg.norm(self.traj_points[-1] - p) > 0.05:
            self.traj_points.append(p)
            if len(self.traj_points) > 1:
                self.traj_item.setData(pos=np.array(self.traj_points))
        
        # Move camera to follow robot if requested
        if self.follow_cb.isChecked():
            # Set the center to the robot position
            self.view.opts['center'] = pg.Vector(p[0], p[1], p[2])
            self.view.update()

    def closeEvent(self, event):
        if self.ros_node:
            self.ros_node.destroy_node()
        super().closeEvent(event)
