#!/usr/bin/env python3

#Wall follow algorithm with Safety node

import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry 
from std_msgs.msg import Float32

class WallFollow(Node):
    def __init__(self):
        super().__init__('wall_follow_node')

        # Subscribers and publishers
        self.sub_scan = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.sub_odom = self.create_subscription(Odometry, '/ego_racecar/odom', self.odom_callback, 10)
        self.pub_throttle = self.create_publisher(Float32, '/accelerator_cmd', 10)
        self.pub_steering = self.create_publisher(Float32, '/steering_cmd', 10)

        # State management
        self.NORMAL_FOLLOWING = 0
        self.OBSTACLE_DETECTED = 1  
        self.AVOIDING = 2
        self.RETURNING = 3
        self.current_state = self.NORMAL_FOLLOWING

        # Obstacle avoidance parameters
        self.obstacle_threshold = 1.65
        self.safe_distance = 2.0
        self.avoidance_angle = np.deg2rad(45)
        self.recovery_distance = 1.0
        self.initial_wall_distance = None
        self.speed = 0.0

        # PID parameters
        self.kp = 1.0
        self.ki = 0
        self.kd = 0.1
        self.error = 0
        self.prev_error = 0
        self.integral = 0
        self.prev_time = 0.0
        self.alpha = None

        # Hardware mapping constants
        self.max_steering_angle = np.deg2rad(50.0)
        self.steering_scale = 2048.0 / self.max_steering_angle
        
        # Messages
        self.throttle_msg = Float32()
        self.steering_msg = Float32()

    def get_range(self, range_data, angle):
        # Hokuyo UST-10LX: 1080 points over 270 degrees
        index = int((angle + 2.356194) / 0.004363323)  # Convert angle to index
        return range_data[min(max(0, index), len(range_data)-1)]

    def get_error(self, range_data, dist):
        theta = np.pi / 6
        b = self.get_range(range_data, np.pi / 2)
        a = self.get_range(range_data, np.pi / 2 - theta)
        
        self.alpha = np.arctan((a * np.cos(theta) - b) / (a * np.sin(theta)))
        
        L = 1.0
        D_t = b * np.cos(self.alpha)
        D_tp1 = D_t + L * np.sin(self.alpha)
        return -1.0 * (dist - D_tp1)

    def map_steering_to_hardware(self, steering_angle):
        return np.clip(steering_angle * self.steering_scale, -2048.0, 2048.0)

    def map_velocity_to_throttle(self, velocity):
        throttle = velocity * 200.0
        if abs(throttle) < 180.0 and throttle != 0:
            throttle = np.sign(throttle) * 170.0
        return np.clip(throttle, -2048.0, 2048.0)

    def odom_callback(self, odom_msg):
        self.speed = odom_msg.twist.twist.linear.x

    def get_safe_velocity(self, front_distance):
        if front_distance < self.obstacle_threshold:
            return 0.3
        elif front_distance < self.safe_distance:
            return 0.4
        return 1.0

    def handle_obstacle_avoidance(self, ranges, error):
        front_idx = len(ranges) // 2
        left_idx = int(3 * len(ranges) // 4)
        right_idx = int(len(ranges) // 4)
    
        front_distance = ranges[front_idx]
        left_distance = ranges[left_idx]
        right_distance = ranges[right_idx]

        if self.current_state == self.NORMAL_FOLLOWING:
            if front_distance < self.safe_distance:
                print(f"Obstacle detected at {front_distance}m")
                self.current_state = self.OBSTACLE_DETECTED
                return -self.avoidance_angle, 0.3
            return None, None

        elif self.current_state == self.OBSTACLE_DETECTED:
            if front_distance >= self.safe_distance:
                print("Path clear - Returning to normal")
                self.current_state = self.NORMAL_FOLLOWING
                return None, None
            return -self.avoidance_angle, 0.3
        return None, None

    def pid_control(self, error, velocity, delta_t, ranges):
        steering_override, velocity_override = self.handle_obstacle_avoidance(ranges, error)
        
        if steering_override is not None:
            steering_angle = steering_override
            velocity = velocity_override
        else:
            self.error = error
            self.integral += self.ki * self.error * delta_t
            steering_angle = (self.kp * self.error + 
                            np.clip(self.integral, -1, +1) + 
                            self.kd * (self.error - self.prev_error) / delta_t)
            self.prev_error = self.error

        self.steering_msg.data = self.map_steering_to_hardware(steering_angle)
        self.throttle_msg.data = self.map_velocity_to_throttle(velocity)
        
        self.pub_steering.publish(self.steering_msg)
        self.pub_throttle.publish(self.throttle_msg)

    def scan_callback(self, scan_msg):
        curr_time = float(scan_msg.header.stamp.sec) + float(scan_msg.header.stamp.nanosec) * 1e-9
        delta_t = curr_time - self.prev_time
   
        ranges = np.array(scan_msg.ranges)
        error = self.get_error(ranges, 0.8)
   
        velocity = 1.5 if abs(self.alpha) <= np.pi/18 else (1.0 if abs(self.alpha) <= np.pi/9 else 0.5)
   
        # Get distances for debugging
        num_points = len(ranges)
        front_idx = num_points // 2
        left_idx = int(3 * num_points // 4) 
        right_idx = int(num_points // 4)

        print(f"State: {self.current_state}")
        print(f"Distances - Front: {ranges[front_idx]:.2f}m, Left: {ranges[left_idx]:.2f}m, Right: {ranges[right_idx]:.2f}m")
        print(f"Velocity: {velocity}, Alpha: {self.alpha:.2f}, Error: {error:.2f}")
        print(f"Steering: {self.steering_msg.data:.2f}, Throttle: {self.throttle_msg.data:.2f}")
        print("------------------------")
   
        self.pid_control(error, velocity, delta_t, ranges)
        self.prev_time = curr_time

def main(args=None):
    rclpy.init(args=args)
    print("WallFollow Initialized")
    wall_follow_node = WallFollow()
    rclpy.spin(wall_follow_node)
    wall_follow_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
