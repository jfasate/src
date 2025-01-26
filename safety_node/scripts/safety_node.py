#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import numpy as np
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32

class SafetyNode(Node):
    """
    The class that handles emergency braking.
    """
    def __init__(self):
        super().__init__('safety_node')
        """
        Two publishers:
        - One publishes to /accelerator_cmd with Float32 for throttle control
        - One publishes to /steering_cmd with Float32 for steering control

        Subscribers:
        - /scan topic for LaserScan messages
        - /ego_racecar/odom topic for current speed
        """
        self.speed = 0.  # v_x
        self.thres = 1.5  # Time-to-collision threshold
        
        # Create ROS subscribers and publishers
        self.sub_scan = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.sub_scan  # prevent unused variable warning
        self.sub_odom = self.create_subscription(Odometry, '/ego_racecar/odom', self.odom_callback, 10)
        self.sub_odom  # prevent unused variable warning
        
        # Publishers for throttle and steering commands
        self.pub_throttle = self.create_publisher(Float32, '/accelerator_cmd', 10)
        self.pub_steering = self.create_publisher(Float32, '/steering_cmd', 10)

    def odom_callback(self, odom_msg):
        # Update current speed
        self.speed = odom_msg.twist.twist.linear.x

    def scan_callback(self, scan_msg):
        # Get the number of points in the scan
        num_points = len(scan_msg.ranges)
        
        # Calculate TTC (Time To Collision)
        r = np.array(scan_msg.ranges)
        # Create theta array with same length as ranges
        theta = np.linspace(scan_msg.angle_min, scan_msg.angle_max, num_points)
        r_dot = self.speed * np.cos(theta)
        ttc = r / np.clip(r_dot, a_min=0.001, a_max=None)  # 0.001 reduces inf & nan
        min_ttc = np.min(np.clip(ttc, 0.0, 60.0))  # clip ttc between 0 ~ 60s
        
        # Publish emergency brake command if needed
        if (self.speed > 0 and min_ttc < self.thres) or (self.speed < 0 and min_ttc < (self.thres + 0.8)):
            print('min_ttc is {}, emergency brake!'.format(round(min_ttc, 2)))
            
            # Send zero throttle command (emergency brake)
            throttle_msg = Float32()
            throttle_msg.data = 0.0
            self.pub_throttle.publish(throttle_msg)
            
            # Keep current steering (don't modify steering during emergency brake)
            # If you want to explicitly set steering to neutral, uncomment below:
            # steering_msg = Float32()
            # steering_msg.data = 0.0
            # self.pub_steering.publish(steering_msg)

def main(args=None):
    rclpy.init(args=args)
    safety_node = SafetyNode()
    rclpy.spin(safety_node)

    # Destroy the node explicitly
    safety_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
