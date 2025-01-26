#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import numpy as np
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32

class WallFollow(Node):
    """ 
    Implement Wall Following on the car
    """
    def __init__(self):
        super().__init__('wall_follow_node')

        lidarscan_topic = '/scan'

        # Create subscribers and publishers
        self.sub_scan = self.create_subscription(LaserScan, lidarscan_topic, self.scan_callback, 10)
        self.sub_scan  # prevent unused variable warning
        
        # Publishers for throttle and steering commands
        self.pub_throttle = self.create_publisher(Float32, '/accelerator_cmd', 10)
        self.pub_steering = self.create_publisher(Float32, '/steering_cmd', 10)

        # PID gains
        self.kp = 1.0
        self.ki = 0
        self.kd = 0.1

        # Store history
        self.error = 0
        self.prev_error = 0
        self.integral = 0
        self.prev_time = 0.0
        self.alpha = None

        # Constants for mapping to hardware commands
        self.max_steering_angle = np.deg2rad(50.0)  # ~0.87 rad
        self.steering_scale = 2048.0 / self.max_steering_angle
        
        # Messages for publishing
        self.throttle_msg = Float32()
        self.steering_msg = Float32()

    def get_range(self, range_data, angle):
        """
        Helper to return the corresponding range measurement at a given angle.
        """
        index = int((np.rad2deg(angle) + 135) * 4 - 1)
        return range_data[index]

    def get_error(self, range_data, dist):
        """
        Calculates the error to the wall. Follow the wall to the left.
        """
        theta = np.pi / 6
        b = self.get_range(range_data, np.pi / 2)
        a = self.get_range(range_data, np.pi / 2 - theta)
        
        self.alpha = np.arctan((a * np.cos(theta) - b) / (a * np.sin(theta)))
        
        L = 1.0  # lookahead distance
        D_t = b * np.cos(self.alpha)
        D_tp1 = D_t + L * np.sin(self.alpha)
        error = -1.0 * (dist - D_tp1)

        return error

    def map_steering_to_hardware(self, steering_angle):
        """
        Map steering angle in radians to hardware steering command
        -2048 (full right) to 2048 (full left)
        """
        steering_angle = np.clip(steering_angle, -self.max_steering_angle, self.max_steering_angle)
        return steering_angle * self.steering_scale

    def map_velocity_to_throttle(self, velocity):
        """
        Map velocity to throttle command
        Hardware range is -2048 to 2048
        Around 170~210 to start moving
        """
        throttle = velocity * 300.0  # Scale factor needs tuning
        if abs(throttle) < 180.0 and throttle != 0:
            throttle = np.sign(throttle) * 85.0
        return np.clip(throttle, -2048.0, 2048.0)

    def pid_control(self, error, velocity, delta_t, front_beam, left_beam, right_beam):
        """
        Based on the calculated error, publish vehicle control
        """
        # PID calculation for steering
        self.error = error
        self.integral += self.ki * self.error * delta_t
        steering_angle = (self.kp * self.error + 
                         np.clip(self.integral, -1, +1) + 
                         self.kd * (self.error - self.prev_error) / delta_t)
        self.prev_error = self.error

        # Emergency steering for front obstacle
        if front_beam < 1.65:
            print(f"Front obstacle detected at {front_beam}m")
            steering_angle = -1 * np.deg2rad(45)
            velocity = 0.2

        # Map to hardware commands
        self.steering_msg.data = self.map_steering_to_hardware(steering_angle)
        self.throttle_msg.data = self.map_velocity_to_throttle(velocity)
        
        # Publish commands
        self.pub_steering.publish(self.steering_msg)
        self.pub_throttle.publish(self.throttle_msg)
        
        # Debug output
        print(f"Steering command: {self.steering_msg.data:.2f}, Throttle command: {self.throttle_msg.data:.2f}")

    def scan_callback(self, scan_msg):
        """
        Callback for LaserScan messages. Calculate error and publish control commands.
        """
        # Calculate time delta
        time_stamp_sec = float(scan_msg.header.stamp.sec)
        time_stamp_nanosec = float(scan_msg.header.stamp.nanosec) * pow(10, -9)
        curr_time = time_stamp_sec + time_stamp_nanosec
        delta_t = curr_time - self.prev_time
        
        # Calculate error for PID
        error = self.get_error(np.array(scan_msg.ranges), 0.8)
        
        # Calculate velocity based on heading angle
        if abs(self.alpha) <= np.pi / 18:    # 10 degrees
            velocity = 1.5
        elif abs(self.alpha) <= np.pi / 9:   # 20 degrees
            velocity = 1.0
        else:
            velocity = 0.5

        # Apply PID control and publish commands
        self.pid_control(error, velocity, delta_t, 
                        scan_msg.ranges[539],  # front beam
                        scan_msg.ranges[899],  # left beam
                        scan_msg.ranges[179])  # right beam

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
