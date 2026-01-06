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
        
        # Grid (Map floor)
        self.grid = gl.GLGridItem()
        self.grid.setSize(50, 50) # Larger grid
        self.grid.setSpacing(1, 1)
        self.grid.setColor((51, 65, 85, 120))
        self.view.addItem(self.grid)
        
        # Lidar Points
        self.scan_item = gl.GLScatterPlotItem(pos=np.array([[0,0,0]]), color=(0.4, 0.5, 1.0, 1.0), size=3, pxMode=True)
        self.view.addItem(self.scan_item)
        
        # Trajectory
        # Initialize with None/Empty to avoid line from origin
        self.traj_item = gl.GLLinePlotItem(pos=np.array([[0,0,0]]), color=(0.1, 0.8, 0.4, 1.0), width=2, antialias=True)
        self.view.addItem(self.traj_item)
        
        # Robot Representation (Axis + Box)
        self.robot_axis = gl.GLAxisItem()
        self.robot_axis.setSize(1.0, 1.0, 1.0) # Larger axes
        self.view.addItem(self.robot_axis)
        
        self.robot_box = gl.GLBoxItem()
        self.robot_box.setSize(0.4, 0.4, 0.2)
        self.robot_box.translate(-0.2, -0.2, 0)
        self.robot_box.setParentItem(self.robot_axis)
        self.robot_box.setColor((59, 130, 246, 200)) # Solid blue body

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
        """Deprecated: Avoid dual pose sources to prevent artifacts"""
        pass

    def update_scan(self, points):
        # Transform scan points from local robot frame to world frame (odom)
        if self.latest_pose:
            px, py = self.latest_pose['x'], self.latest_pose['y']
            # Only use Yaw for stable 2D visualization
            qw, qx, qy, qz = self.latest_pose['qw'], self.latest_pose['qx'], self.latest_pose['qy'], self.latest_pose['qz']
            
            # Simple 2D rotation (Yaw) from quaternion
            siny_cosp = 2 * (qw * qz + qx * qy)
            cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
            yaw = math.atan2(siny_cosp, cosy_cosp)
            
            R = np.array([[np.cos(yaw), -np.sin(yaw), 0],
                          [np.sin(yaw),  np.cos(yaw), 0],
                          [0,            0,           1]])
            
            # Project scan slightly above ground
            world_points = points @ R.T + np.array([px, py, 0.05])
            self.scan_item.setData(pos=world_points)
        else:
            self.scan_item.setData(pos=points)

    def update_pose(self, pose):
        self.latest_pose = pose
        # Force Z=0 for ground plane visualization
        p_ground = np.array([pose['x'], pose['y'], 0.0])
        
        # Build 4x4 Transformation Matrix
        tr = pg.Transform3D()
        tr.translate(p_ground[0], p_ground[1], p_ground[2])
        
        # Only use Yaw for stable 2D Cap
        qw, qx, qy, qz = pose['qw'], pose['qx'], pose['qy'], pose['qz']
        siny_cosp = 2 * (qw * qz + qx * qy)
        cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
        yaw_deg = math.degrees(math.atan2(siny_cosp, cosy_cosp))
        tr.rotate(yaw_deg, 0, 0, 1)
        
        # Apply transform to the axis (and its child box)
        self.robot_axis.setTransform(tr)
        
        # Update Trajectory (Stored with Z=0)
        # 1. Skip if the pose is at the exact origin (avoid start noise)
        if abs(p_ground[0]) < 0.001 and abs(p_ground[1]) < 0.001:
            return

        # 2. Initialize if empty
        if not self.traj_points:
             # Use current position as starting point (not 0,0,0)
             self.traj_points.append(p_ground)
             return

        # 3. Filter distance to previous point
        dist = np.linalg.norm(self.traj_points[-1] - p_ground)
        # If we jump more than 2 meters instantly, it's a simulation spike - ignore it
        if dist > 2.0:
            return

        if np.linalg.norm(self.traj_points[-1] - p_ground) > 0.05:
            # Detect huge time jump or new run start (distance > 5m)
            if np.linalg.norm(self.traj_points[-1] - p_ground) > 5.0:
                self.traj_points = [p_ground]
            else:
                self.traj_points.append(p_ground)
            
            # Limit history
            if len(self.traj_points) > 5000:
                self.traj_points.pop(0)
            
            if len(self.traj_points) > 1:
                self.traj_item.setData(pos=np.array(self.traj_points, dtype=np.float32))
        
        # Move camera to follow robot if requested
        if self.follow_cb.isChecked():
            # Set the center to the robot position
            self.view.opts['center'] = pg.Vector(p_ground[0], p_ground[1], p_ground[2])
            self.view.update()

    def closeEvent(self, event):
        if self.ros_node:
            self.ros_node.destroy_node()
        super().closeEvent(event)
