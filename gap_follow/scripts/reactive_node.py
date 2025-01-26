#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import numpy as np
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32

class ReactiveFollowGap(Node):
    """ 
    Implement Wall Following on the car
    This is just a template, you are free to implement your own node!
    """
    def __init__(self):
        super().__init__('reactive_node')
        
        # Topics & Subs, Pubs
        lidarscan_topic = '/scan'

        # Subscribe to LIDAR
        self.sub_scan = self.create_subscription(LaserScan, lidarscan_topic, self.lidar_callback, 10)
        self.sub_scan  # prevent unused variable warning
        
        # Publishers for throttle and steering commands
        self.pub_throttle = self.create_publisher(Float32, '/accelerator_cmd', 10)
        self.pub_steering = self.create_publisher(Float32, '/steering_cmd', 10)
        
        # Messages for publishing
        self.throttle_msg = Float32()
        self.steering_msg = Float32()
        
        # params
        self.downsample_gap = 10
        self.max_sight = 4.0
        self.bubble_radius = 2
        self.extender_thres = 0.5
        self.max_gap_safe_dist = 1.2
        
        # Constants for mapping steering angles to hardware commands
        # 2048.0 for full left, -2048.0 for full right
        # 50 degrees max steering angle (100 degrees end-to-end)
        self.max_steering_angle = np.deg2rad(50.0)  # ~0.87 rad
        self.steering_scale = 2048.0 / self.max_steering_angle

    def preprocess_lidar(self, ranges):
        """ 
        Preprocess the LiDAR scan array. Expert implementation includes:
            1.Setting each value to the mean over some window
            2.Rejecting high values (eg. > 3m)
        """
        proc_ranges = np.zeros(int(720 / self.downsample_gap))
        for i in range(len(proc_ranges)):
            proc_ranges[i] = sum(ranges[i * self.downsample_gap : (i + 1) * self.downsample_gap]) / self.downsample_gap
        
        proc_ranges = np.clip(proc_ranges, 0.0, self.max_sight)
        
        return proc_ranges

    def disparity_extender(self, proc_ranges):
        i = 0
        while i < len(proc_ranges) - 1:
            if proc_ranges[i + 1] - proc_ranges[i] >= self.extender_thres:
                proc_ranges[i : min(i + self.bubble_radius + 1, len(proc_ranges))] = proc_ranges[i]
                i += self.bubble_radius + 1
            elif proc_ranges[i] - proc_ranges[i + 1] >= self.extender_thres:
                proc_ranges[max(0, i - self.bubble_radius + 1) : i + 1] = proc_ranges[i + 1]
                i += self.bubble_radius + 1
            else:
                i += 1

        return proc_ranges

    def find_max_gap(self, free_space_ranges):
        """ 
        Return the start index & end index of the max gap in free_space_ranges
        """
        longest_streak = 0
        streak = 0
        end_index = 0
        start_index = 0

        for i in range(len(free_space_ranges)):
            if free_space_ranges[i] > self.max_gap_safe_dist:
                streak += 1
                if (streak > longest_streak):
                    longest_streak = streak
                    end_index = i + 1
                    start_index = end_index - longest_streak
            else:
                streak = 0

        return start_index, end_index

    def find_best_point(self, start_i, end_i, ranges):
        """
        Start_i & end_i are start and end indicies of max-gap range, respectively
        Return index of best point in ranges
        """
        best_index = (start_i + end_i) / 2
        return best_index

    def find_deepest_point(self, start_i, end_i, ranges):
        """
        Start_i & end_i are start and end indicies of max-gap range, respectively
        Return index of deepest point in ranges
        """
        deepest_index = np.argmax(ranges[start_i:end_i])
        deepest_index += start_i
        return deepest_index

    def map_steering_to_hardware(self, steering_angle):
        """
        Map steering angle in radians to hardware steering command
        Positive steering_angle is left turn, negative is right turn
        Hardware range is -2048 (full right) to 2048 (full left)
        """
        # Clip the steering angle to max physical angle
        steering_angle = np.clip(steering_angle, -self.max_steering_angle, self.max_steering_angle)
        # Map to hardware range
        return steering_angle * self.steering_scale

    def map_velocity_to_throttle(self, velocity):
        """
        Map velocity to throttle command
        Hardware range is -2048 to 2048
        Around 170~210 to start moving
        """
        # Simple linear mapping, adjust these values based on testing
        throttle = velocity * 300.0  # This scaling factor needs tuning
        # Ensure minimum throttle to overcome dead zone
        if abs(throttle) < 180.0 and throttle != 0:
            throttle = np.sign(throttle) * 180.0
        return np.clip(throttle, -2048.0, 2048.0)

    def lidar_callback(self, data):
        """ 
        Process each LiDAR scan as per the Follow Gap algorithm & publish throttle and steering commands
        """
        # preprocessing & downsampling
        ranges = np.array(data.ranges[180:899])
        proc_ranges = self.preprocess_lidar(ranges)
        print("downsampling = ", proc_ranges)

        # find best gap considering multiple factors
        start_max_gap, end_max_gap = self.find_best_gap(proc_ranges)
        print('start_best_gap = ', start_max_gap)
        print('end_best_gap = ', end_max_gap)

        # find the best point in the gap 
        best_i = self.find_best_point(start_max_gap, end_max_gap, proc_ranges)
        print(f'best point:', best_i)

        # Calculate steering angle
        steering_angle = np.deg2rad(best_i * self.downsample_gap / 4.0 - 90.0)

        # Calculate velocity
        if sum(data.ranges[530:549]) / 20 < 2.0:
            velocity = 2.0
        else:
            velocity = 5.0
        print(f'velocity:', velocity)

        # Map to hardware commands and publish
        self.steering_msg.data = self.map_steering_to_hardware(steering_angle)
        self.throttle_msg.data = self.map_velocity_to_throttle(velocity)
        
        # Publish messages
        self.pub_steering.publish(self.steering_msg)
        self.pub_throttle.publish(self.throttle_msg)
        
        print(f'steering command:', self.steering_msg.data)
        print(f'throttle command:', self.throttle_msg.data)


def main(args=None):
    rclpy.init(args=args)
    print("ReactiveFollowGap Initialized")
    reactive_node = ReactiveFollowGap()
    rclpy.spin(reactive_node)

    reactive_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
