#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import numpy as np

class SensorDegrader(Node):
    def __init__(self):
        super().__init__('sensor_degrader')
        
        # Parameters
        self.declare_parameter('max_range', 10.0)
        self.declare_parameter('noise_std', 0.0)
        self.declare_parameter('speed_scale', 1.0)
        self.declare_parameter('enabled', False)

        self.max_range = self.get_parameter('max_range').value
        self.noise_std = self.get_parameter('noise_std').value
        self.speed_scale = self.get_parameter('speed_scale').value
        self.enabled = self.get_parameter('enabled').value

        if not self.enabled:
            self.get_logger().info("Sensor Degrader is DISABLED.")
            return

        self.get_logger().info(f"Sensor Degrader ENABLED: range={self.max_range}m, noise={self.noise_std}, speed_scale={self.speed_scale}")

        # Scan subscriber/publisher
        self.scan_sub = self.create_subscription(LaserScan, '/scan_raw', self.scan_callback, 10)
        self.scan_pub = self.create_publisher(LaserScan, '/scan', 10)

        # Cmd_vel subscriber/publisher (to limit speed)
        self.cmd_sub = self.create_subscription(Twist, '/cmd_vel_raw', self.cmd_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

    def scan_callback(self, msg):
        # 1. Clip range
        ranges = np.array(msg.ranges)
        ranges[ranges > self.max_range] = float('inf')
        
        # 2. Add noise
        if self.noise_std > 0:
            noise = np.random.normal(0, self.noise_std, ranges.shape)
            # Only add noise to valid (non-inf) readings
            mask = np.isfinite(ranges)
            ranges[mask] += noise[mask]

        msg.ranges = ranges.tolist()
        self.scan_pub.publish(msg)

    def cmd_callback(self, msg):
        # 3. Scale speed
        msg.linear.x *= self.speed_scale
        msg.linear.y *= self.speed_scale
        msg.angular.z *= self.speed_scale
        self.cmd_pub.publish(msg)

def main():
    rclpy.init()
    node = SensorDegrader()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
