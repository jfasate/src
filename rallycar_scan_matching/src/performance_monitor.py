#!/usr/bin/env python3

"""
Simple performance monitoring for scan matching system
Monitors frequencies and provides console output
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
import time
from collections import deque


class SimplePerformanceMonitor(Node):
    """
    Simple performance monitor for scan matching system
    """
    
    def __init__(self):
        super().__init__('performance_monitor')
        
        # Frequency tracking
        self.scan_times = deque(maxlen=50)
        self.odom_times = deque(maxlen=50)
        self.pose_times = deque(maxlen=50)
        
        # Subscribers
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/scan_match_odom', self.odom_callback, 10)
        self.pose_sub = self.create_subscription(
            PoseStamped, '/scan_match_location', self.pose_callback, 10)
        
        # Timer for status reporting
        self.status_timer = self.create_timer(5.0, self.report_status)
        
        self.get_logger().info("Performance Monitor started")
    
    def scan_callback(self, msg):
        """Record scan message timestamp"""
        self.scan_times.append(time.time())
    
    def odom_callback(self, msg):
        """Record odometry message timestamp"""
        self.odom_times.append(time.time())
    
    def pose_callback(self, msg):
        """Record pose message timestamp"""
        self.pose_times.append(time.time())
    
    def calculate_frequency(self, times):
        """Calculate frequency from timestamp deque"""
        if len(times) < 2:
            return 0.0
        
        time_span = times[-1] - times[0]
        if time_span <= 0:
            return 0.0
        
        return (len(times) - 1) / time_span
    
    def report_status(self):
        """Report performance status"""
        scan_freq = self.calculate_frequency(self.scan_times)
        odom_freq = self.calculate_frequency(self.odom_times)
        pose_freq = self.calculate_frequency(self.pose_times)
        
        self.get_logger().info(
            f"Performance Status: "
            f"LiDAR: {scan_freq:.1f}Hz, "
            f"Odom: {odom_freq:.1f}Hz, "
            f"Pose: {pose_freq:.1f}Hz"
        )
        
        # Warnings for low performance
        if odom_freq < 15.0 and odom_freq > 0:
            self.get_logger().warn(f"Low scan matching frequency: {odom_freq:.1f}Hz")
        if scan_freq < 30.0 and scan_freq > 0:
            self.get_logger().warn(f"Low LiDAR frequency: {scan_freq:.1f}Hz")


def main(args=None):
    rclpy.init(args=args)
    monitor = SimplePerformanceMonitor()
    
    try:
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        pass
    finally:
        monitor.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
