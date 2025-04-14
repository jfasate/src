#include <sstream>
#include <string>
#include <cmath>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/color_rgba.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

// Include the scan matching utilities - FIXED PATHS
#include "rallycar_scan_matching/correspond.h"
#include "rallycar_scan_matching/transform.h"
#include "rallycar_scan_matching/visualization.h"

using namespace std;

// Topic configurations - adaptable to your rallycar setup
const string &TOPIC_SCAN = "/scan";
const string &TOPIC_ODOM_OUT = "/scan_match_odom";
const string &TOPIC_POSE_OUT = "/scan_match_location";
const string &TOPIC_RVIZ = "/scan_match_debug";
const string &FRAME_MAP = "map";
const string &FRAME_BASE = "base_link";
const string &FRAME_LASER = "laser";

// Algorithm parameters - tune these for your hardware
const float RANGE_LIMIT = 10.0;      // Maximum valid range for LiDAR
const float MAX_ITER = 20.0;         // Maximum algorithm iterations
const float MIN_INFO = 0.1;          // Minimum information value
const float A = (1 - MIN_INFO) / MAX_ITER / MAX_ITER;
const float MOTION_THRESHOLD = 0.01; // Minimum movement to register as motion

class ScanMatch : public rclcpp::Node
{
private:
    // Publishers
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    
    // Subscribers
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
    
    // TF broadcaster
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    
    // Data storage
    vector<Point> current_points_;
    vector<Point> transformed_points_;
    vector<Point> prev_points_;
    vector<Correspondence> corresponds_;
    vector<vector<int>> jump_table_;
    
    // Transform storage
    Transform prev_trans_, curr_trans_;
    Transform global_transform_; // Accumulated global transform
    
    // Visualization
    PointVisualizer *points_viz_;
    
    // Messages
    geometry_msgs::msg::PoseStamped pose_msg_;
    nav_msgs::msg::Odometry odom_msg_;
    
    // Parameters
    bool publish_tf_;
    bool use_fast_correspondence_;
    double min_scan_distance_;
    bool point_cloud_visualization_;
    
    // Timing
    rclcpp::Time last_scan_time_;
    
public:
    ScanMatch() : Node("scanmatch_node")
    {
        // Initialize parameters
        this->declare_parameter<bool>("publish_tf", true);
        this->declare_parameter<bool>("use_fast_correspondence", true);
        this->declare_parameter<double>("min_scan_distance", 0.2);
        this->declare_parameter<bool>("point_cloud_visualization", true);
        
        publish_tf_ = this->get_parameter("publish_tf").as_bool();
        use_fast_correspondence_ = this->get_parameter("use_fast_correspondence").as_bool();
        min_scan_distance_ = this->get_parameter("min_scan_distance").as_double();
        point_cloud_visualization_ = this->get_parameter("point_cloud_visualization").as_bool();
        
        // Initialize publishers
        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(TOPIC_POSE_OUT, 10);
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(TOPIC_ODOM_OUT, 10);
        marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(TOPIC_RVIZ, 10);
        
        // Initialize subscriber
        laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            TOPIC_SCAN, 10, std::bind(&ScanMatch::handleLaserScan, this, std::placeholders::_1));
        
        // Initialize TF broadcaster
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        
        // Initialize visualization
        if (point_cloud_visualization_) {
            points_viz_ = new PointVisualizer(marker_pub_, "scan_match", FRAME_LASER);
        }
        
        // Initialize messages
        initializeMessages();
        
        // Initialize transform
        global_transform_ = Transform();
        
        RCLCPP_INFO(this->get_logger(), "Scan matching node initialized");
    }
    
    ~ScanMatch() {
        if (point_cloud_visualization_ && points_viz_) {
            delete points_viz_;
        }
    }
    
private:
    void initializeMessages() {
        // Initialize pose message
        pose_msg_.header.frame_id = FRAME_MAP;
        pose_msg_.pose.position.z = 0.0;
        
        // Initialize odometry message
        odom_msg_.header.frame_id = FRAME_MAP;
        odom_msg_.child_frame_id = FRAME_BASE;
        odom_msg_.pose.pose.position.z = 0.0;
        
        // Set covariance (diagonal)
        for (int i = 0; i < 36; ++i) {
            odom_msg_.pose.covariance[i] = 0.0;
            odom_msg_.twist.covariance[i] = 0.0;
        }
        // Position covariance
        odom_msg_.pose.covariance[0] = 0.01;  // x
        odom_msg_.pose.covariance[7] = 0.01;  // y
        odom_msg_.pose.covariance[35] = 0.01; // yaw
        
        // Velocity covariance
        odom_msg_.twist.covariance[0] = 0.1;   // vx
        odom_msg_.twist.covariance[7] = 0.1;   // vy
        odom_msg_.twist.covariance[35] = 0.05; // vyaw
    }
    
    void handleLaserScan(const sensor_msgs::msg::LaserScan::ConstSharedPtr msg) {
        // Extract points from laser scan
        readScan(msg);
        
        // If this is the first scan, just store it
        if (prev_points_.empty()) {
            RCLCPP_INFO(this->get_logger(), "First scan received, initializing");
            prev_points_ = current_points_;
            last_scan_time_ = this->now();
            return;
        }
        
        // Calculate time difference since last scan
        rclcpp::Time current_time = this->now();
        double dt = (current_time - last_scan_time_).seconds();
        last_scan_time_ = current_time;
        
        // Visualize previous scan points
        if (point_cloud_visualization_) {
            std_msgs::msg::ColorRGBA col;
            col.r = 1.0; col.g = 0.0; col.b = 0.0; col.a = 1.0;
            points_viz_->addPoints(prev_points_, col);
        }
        
        // Compute jump table for correspondence search
        computeJump(jump_table_, prev_points_);
        
        // Initialize transform and iteration counter
        curr_trans_ = Transform();
        int count = 0;
        
        // Iterative closest point algorithm
        while (count < MAX_ITER && (curr_trans_ != prev_trans_ || count == 0)) {
            // Transform current points using current transform estimate
            transformPoints(current_points_, curr_trans_, transformed_points_);
            
            // Find correspondence between points
            if (use_fast_correspondence_) {
                getCorrespondence(prev_points_, transformed_points_, current_points_, 
                                  jump_table_, corresponds_, A * count * count + MIN_INFO);
            } else {
                getNaiveCorrespondence(prev_points_, transformed_points_, current_points_, 
                                       jump_table_, corresponds_, A * count * count + MIN_INFO);
            }
            
            // Save previous transform
            prev_trans_ = curr_trans_;
            
            // Update transform based on correspondences
            updateTransform(corresponds_, curr_trans_);
            
            ++count;
        }
        
        // Visualize transformed points
        if (point_cloud_visualization_) {
            std_msgs::msg::ColorRGBA col;
            col.r = 0.0; col.g = 1.0; col.b = 0.0; col.a = 1.0;
            points_viz_->addPoints(transformed_points_, col);
            points_viz_->publishPoints();
        }
        
        RCLCPP_DEBUG(this->get_logger(), "ICP iterations: %i", count);
        
        // Update global transform
        global_transform_ = global_transform_ + curr_trans_;
        
        // Publish results
        publishTransform(dt);
        
        // Update previous points
        prev_points_ = current_points_;
    }
    
    void readScan(const sensor_msgs::msg::LaserScan::ConstSharedPtr msg) {
        float range_min = msg->range_min;
        float range_max = msg->range_max;
        float angle_min = msg->angle_min;
        float angle_increment = msg->angle_increment;
        
        const vector<float> &ranges = msg->ranges;
        current_points_.clear();
        
        for (int i = 0; i < ranges.size(); ++i) {
            float range = ranges.at(i);
            
            // Filter invalid readings
            if (range > RANGE_LIMIT || isnan(range) || range < range_min || range > range_max) {
                continue;
            }
            
            // Add valid points
            current_points_.push_back(Point(range, angle_min + angle_increment * i));
        }
        
        // Filter points to achieve a more uniform distribution
        // This can be enabled if the LiDAR has too many points and slows down processing
        // filterPoints();
    }
    
    void publishTransform(double dt) {
        // Create TF transform
        geometry_msgs::msg::TransformStamped transform_stamped;
        transform_stamped.header.stamp = this->now();
        transform_stamped.header.frame_id = FRAME_MAP;
        transform_stamped.child_frame_id = FRAME_BASE;
        
        // Set translation
        transform_stamped.transform.translation.x = global_transform_.x_disp;
        transform_stamped.transform.translation.y = global_transform_.y_disp;
        transform_stamped.transform.translation.z = 0.0;
        
        // Set rotation (convert from Euler angle to quaternion)
        tf2::Quaternion q;
        q.setRPY(0, 0, global_transform_.theta_rot);
        transform_stamped.transform.rotation.x = q.x();
        transform_stamped.transform.rotation.y = q.y();
        transform_stamped.transform.rotation.z = q.z();
        transform_stamped.transform.rotation.w = q.w();
        
        // Broadcast transform if enabled
        if (publish_tf_) {
            tf_broadcaster_->sendTransform(transform_stamped);
        }
        
        // Update pose message
        pose_msg_.header.stamp = this->now();
        pose_msg_.pose.position.x = global_transform_.x_disp;
        pose_msg_.pose.position.y = global_transform_.y_disp;
        pose_msg_.pose.orientation = transform_stamped.transform.rotation;
        
        // Update odometry message
        odom_msg_.header.stamp = this->now();
        odom_msg_.pose.pose = pose_msg_.pose;
        
        // Calculate twist (velocity) from the current transform and time difference
        if (dt > 0) {
            odom_msg_.twist.twist.linear.x = curr_trans_.x_disp / dt;
            odom_msg_.twist.twist.linear.y = curr_trans_.y_disp / dt;
            odom_msg_.twist.twist.angular.z = curr_trans_.theta_rot / dt;
        }
        
        // Publish messages
        pose_pub_->publish(pose_msg_);
        odom_pub_->publish(odom_msg_);
    }
    
    // Optional: Filter points to reduce computation if needed
    void filterPoints() {
        if (current_points_.size() <= 100) return; // Don't filter if few points
        
        vector<Point> filtered_points;
        filtered_points.reserve(current_points_.size() / 2);
        
        for (int i = 0; i < current_points_.size(); i += 2) {
            filtered_points.push_back(current_points_[i]);
        }
        
        current_points_ = filtered_points;
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ScanMatch>());
    rclcpp::shutdown();
    return 0;
}
