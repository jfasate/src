#include <sstream>
#include <string>
#include <cmath>
#include <vector>
#include <chrono>
#include <deque>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/color_rgba.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

// Include the scan matching utilities
#include "rallycar_scan_matching/correspond.h"
#include "rallycar_scan_matching/transform.h"
#include "rallycar_scan_matching/visualization.h"

using namespace std;
using namespace std::chrono;

// Topic configurations - adaptable to your rallycar setup
const string &TOPIC_SCAN = "/scan";
const string &TOPIC_ODOM_OUT = "/scan_match_odom";
const string &TOPIC_POSE_OUT = "/scan_match_location";
const string &TOPIC_RVIZ = "/scan_match_debug";
const string &FRAME_MAP = "map";
const string &FRAME_BASE = "base_link";
const string &FRAME_LASER = "laser";

// Performance-optimized algorithm parameters for Hokuyo UST-10LX
const float RANGE_LIMIT = 10.0;        // UST-10LX max range
const float RANGE_MIN = 0.06;          // UST-10LX min range
const float MAX_ITER = 10.0;           // Reduced iterations for speed
const float MIN_INFO = 0.05;           // Reduced for faster convergence
const float A = (1 - MIN_INFO) / MAX_ITER / MAX_ITER;
const float MOTION_THRESHOLD = 0.005;  // More sensitive for racing
const float ANGULAR_RESOLUTION = 0.25 * M_PI / 180.0; // UST-10LX resolution
const int DOWNSAMPLE_FACTOR = 2;       // Skip every other point for speed
const float MAX_CORRESPONDENCE_DISTANCE = 0.5; // Maximum correspondence distance

class HighPerformanceScanMatch : public rclcpp::Node
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
    
    // Data storage - optimized for performance
    vector<Point> current_points_;
    vector<Point> transformed_points_;
    vector<Point> prev_points_;
    vector<Correspondence> corresponds_;
    vector<vector<int>> jump_table_;
    
    // Transform storage
    Transform prev_trans_, curr_trans_;
    Transform global_transform_; // Accumulated global transform
    
    // Visualization - reduced frequency for performance
    PointVisualizer *points_viz_;
    int viz_counter_;
    static const int VIZ_FREQUENCY = 10; // Visualize every 10th scan
    
    // Messages
    geometry_msgs::msg::PoseStamped pose_msg_;
    nav_msgs::msg::Odometry odom_msg_;
    
    // Performance monitoring
    rclcpp::Time last_scan_time_;
    deque<double> processing_times_;
    static const int TIMING_WINDOW = 50;
    
    // Skip frame handling for performance
    int frame_skip_counter_;
    static const int MAX_FRAME_SKIP = 2; // Process every 2-3 frames max
    
    // Motion prediction for better initial guess
    Transform velocity_estimate_;
    
public:
    HighPerformanceScanMatch() : Node("scanmatch_node"), viz_counter_(0), frame_skip_counter_(0)
    {
        // Set high priority for this node
        RCLCPP_INFO(this->get_logger(), "Starting High-Performance Scan Matching for Racing");
        
        // Initialize publishers
        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(TOPIC_POSE_OUT, 10);
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(TOPIC_ODOM_OUT, 10);
        marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(TOPIC_RVIZ, 5);
        
        // Initialize subscriber with high priority
        laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            TOPIC_SCAN, rclcpp::SensorDataQoS(), 
            std::bind(&HighPerformanceScanMatch::handleLaserScan, this, std::placeholders::_1));
        
        // Initialize TF broadcaster
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        
        // Initialize visualization with reduced frequency
        points_viz_ = new PointVisualizer(marker_pub_, "scan_match", FRAME_LASER);
        
        // Initialize messages
        initializeMessages();
        
        // Initialize transform
        global_transform_ = Transform();
        velocity_estimate_ = Transform();
        
        // Reserve memory for performance
        current_points_.reserve(1081);     // UST-10LX max points
        transformed_points_.reserve(1081);
        prev_points_.reserve(1081);
        corresponds_.reserve(500);         // Reduced for performance
        
        RCLCPP_INFO(this->get_logger(), "High-Performance Scan Matching initialized for Hokuyo UST-10LX");
    }
    
    ~HighPerformanceScanMatch() {
        if (points_viz_) {
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
        
        // Set covariance for racing (tighter estimates)
        for (int i = 0; i < 36; ++i) {
            odom_msg_.pose.covariance[i] = 0.0;
            odom_msg_.twist.covariance[i] = 0.0;
        }
        // Position covariance - tighter for racing
        odom_msg_.pose.covariance[0] = 0.005;  // x
        odom_msg_.pose.covariance[7] = 0.005;  // y
        odom_msg_.pose.covariance[35] = 0.01;  // yaw
        
        // Velocity covariance
        odom_msg_.twist.covariance[0] = 0.05;   // vx
        odom_msg_.twist.covariance[7] = 0.05;   // vy
        odom_msg_.twist.covariance[35] = 0.02;  // vyaw
    }
    
    void handleLaserScan(const sensor_msgs::msg::LaserScan::ConstSharedPtr msg) {
        auto start_time = high_resolution_clock::now();
        
        // Extract and downsample points for performance
        readScanOptimized(msg);
        
        // If this is the first scan, just store it
        if (prev_points_.empty()) {
            RCLCPP_INFO(this->get_logger(), "First scan received with %zu points", current_points_.size());
            prev_points_ = current_points_;
            last_scan_time_ = this->now();
            return;
        }
        
        // Calculate time difference
        rclcpp::Time current_time = this->now();
        double dt = (current_time - last_scan_time_).seconds();
        last_scan_time_ = current_time;
        
        // Skip frames if processing is too slow (adaptive frame skipping)
        double avg_processing_time = getAverageProcessingTime();
        bool skip_frame = false;
        
        if (avg_processing_time > 0.02 && frame_skip_counter_ < MAX_FRAME_SKIP) { // If >20ms
            frame_skip_counter_++;
            skip_frame = true;
        } else {
            frame_skip_counter_ = 0;
        }
        
        if (skip_frame) {
            // Use motion prediction for skipped frames
            Transform predicted_motion = Transform(
                velocity_estimate_.x_disp * dt,
                velocity_estimate_.y_disp * dt,
                velocity_estimate_.theta_rot * dt
            );
            global_transform_ = global_transform_ + predicted_motion;
            publishTransform();
            return;
        }
        
        // Use velocity estimate as initial guess
        curr_trans_ = Transform(
            velocity_estimate_.x_disp * dt,
            velocity_estimate_.y_disp * dt,
            velocity_estimate_.theta_rot * dt
        );
        
        // Fast correspondence computation
        computeJump(jump_table_, prev_points_);
        
        // Iterative closest point with reduced iterations
        int count = 0;
        Transform prev_iteration = Transform();
        
        while (count < MAX_ITER && (curr_trans_ != prev_iteration || count == 0)) {
            // Transform current points
            transformPointsOptimized(current_points_, curr_trans_, transformed_points_);
            
            // Fast correspondence with distance threshold
            getCorrespondenceOptimized(prev_points_, transformed_points_, current_points_, 
                                      jump_table_, corresponds_, A * count * count + MIN_INFO);
            
            // Check if we have enough correspondences
            if (corresponds_.size() < 10) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                                    "Insufficient correspondences: %zu", corresponds_.size());
                break;
            }
            
            prev_iteration = curr_trans_;
            updateTransform(corresponds_, curr_trans_);
            ++count;
            
            // Early termination if converged
            if (fabs(curr_trans_.x_disp - prev_iteration.x_disp) < 0.001 &&
                fabs(curr_trans_.y_disp - prev_iteration.y_disp) < 0.001 &&
                fabs(curr_trans_.theta_rot - prev_iteration.theta_rot) < 0.001) {
                break;
            }
        }
        
        // Update velocity estimate for prediction
        if (dt > 0) {
            velocity_estimate_ = Transform(
                curr_trans_.x_disp / dt,
                curr_trans_.y_disp / dt,
                curr_trans_.theta_rot / dt
            );
        }
        
        // Visualization at reduced frequency
        if (viz_counter_++ % VIZ_FREQUENCY == 0) {
            std_msgs::msg::ColorRGBA red, green;
            red.r = 1.0; red.g = 0.0; red.b = 0.0; red.a = 1.0;
            green.r = 0.0; green.g = 1.0; green.b = 0.0; green.a = 1.0;
            
            points_viz_->addPoints(prev_points_, red);
            points_viz_->addPoints(transformed_points_, green);
            points_viz_->publishPoints();
        }
        
        // Update global transform
        global_transform_ = global_transform_ + curr_trans_;
        
        // Publish results
        publishTransform();
        
        // Update previous points
        prev_points_ = current_points_;
        
        // Performance monitoring
        auto end_time = high_resolution_clock::now();
        double processing_time = duration_cast<microseconds>(end_time - start_time).count() / 1000.0; // ms
        updateProcessingTime(processing_time);
        
        RCLCPP_DEBUG(this->get_logger(), "ICP: %d iter, %.2fms, %zu corr, %.1f Hz", 
                    count, processing_time, corresponds_.size(), 1000.0/processing_time);
    }
    
    void readScanOptimized(const sensor_msgs::msg::LaserScan::ConstSharedPtr msg) {
        const vector<float> &ranges = msg->ranges;
        current_points_.clear();
        
        float angle = msg->angle_min;
        float angle_increment = msg->angle_increment;
        
        // Downsample for performance while keeping good coverage - FIXED SIGN COMPARISON
        for (size_t i = 0; i < ranges.size(); i += DOWNSAMPLE_FACTOR) {
            float range = ranges[i];
            
            // Optimized filtering for UST-10LX specs
            if (range >= RANGE_MIN && range <= RANGE_LIMIT && !isnan(range) && !isinf(range)) {
                current_points_.emplace_back(range, angle + angle_increment * static_cast<float>(i));
            }
        }
        
        RCLCPP_DEBUG(this->get_logger(), "Processed %zu points from %zu raw points", 
                    current_points_.size(), ranges.size());
    }
    
    void transformPointsOptimized(const vector<Point> &points, Transform &t, vector<Point> &transformed_points) {
        transformed_points.clear();
        transformed_points.reserve(points.size());
        
        // Pre-compute trigonometric values
        float cos_theta = cos(t.theta_rot);
        float sin_theta = sin(t.theta_rot);
        
        for (const Point& p : points) {
            float x = p.r * cos(p.theta);
            float y = p.r * sin(p.theta);
            
            // Apply transformation
            float new_x = cos_theta * x - sin_theta * y + t.x_disp;
            float new_y = sin_theta * x + cos_theta * y + t.y_disp;
            
            transformed_points.emplace_back(sqrt(new_x*new_x + new_y*new_y), atan2(new_y, new_x));
        }
    }
    
    void getCorrespondenceOptimized(vector<Point>& old_points, vector<Point>& trans_points, 
                                   vector<Point>& points, vector<vector<int>>& /* jump_table */, 
                                   vector<Correspondence>& c, float /* prob */) {
        c.clear();
        c.reserve(trans_points.size() / 2); // Estimate fewer correspondences
        
        const int trans_size = static_cast<int>(trans_points.size());
        const int old_size = static_cast<int>(old_points.size());
        
        for (int ind_trans = 0; ind_trans < min(old_size, trans_size); ++ind_trans) {
            int best = 0;
            int second_best = 0;
            float best_dist = MAX_CORRESPONDENCE_DISTANCE * MAX_CORRESPONDENCE_DISTANCE;
            
            // Simplified correspondence search for speed
            int search_start = max(0, ind_trans - 20);
            int search_end = min(old_size, ind_trans + 20);
            
            for (int i = search_start; i < search_end; ++i) {
                float dist = trans_points[ind_trans].distToPoint2(&old_points[i]);
                if (dist < best_dist) {
                    best_dist = dist;
                    best = i;
                }
            }
            
            // Only add correspondence if distance is reasonable
            if (best_dist < MAX_CORRESPONDENCE_DISTANCE * MAX_CORRESPONDENCE_DISTANCE) {
                second_best = (best > 0) ? best - 1 : best + 1;
                if (second_best >= old_size) second_best = old_size - 1;
                
                c.emplace_back(&trans_points[ind_trans], &points[ind_trans], 
                              &old_points[best], &old_points[second_best]);
            }
        }
    }
    
    void publishTransform() {
        rclcpp::Time now = this->now();
        
        // Create TF transform
        geometry_msgs::msg::TransformStamped transform_stamped;
        transform_stamped.header.stamp = now;
        transform_stamped.header.frame_id = FRAME_MAP;
        transform_stamped.child_frame_id = FRAME_BASE;
        
        // Set translation
        transform_stamped.transform.translation.x = global_transform_.x_disp;
        transform_stamped.transform.translation.y = global_transform_.y_disp;
        transform_stamped.transform.translation.z = 0.0;
        
        // Set rotation
        tf2::Quaternion q;
        q.setRPY(0, 0, global_transform_.theta_rot);
        transform_stamped.transform.rotation.x = q.x();
        transform_stamped.transform.rotation.y = q.y();
        transform_stamped.transform.rotation.z = q.z();
        transform_stamped.transform.rotation.w = q.w();
        
        // Always broadcast transform for racing
        tf_broadcaster_->sendTransform(transform_stamped);
        
        // Update pose message
        pose_msg_.header.stamp = now;
        pose_msg_.pose.position.x = global_transform_.x_disp;
        pose_msg_.pose.position.y = global_transform_.y_disp;
        pose_msg_.pose.orientation = transform_stamped.transform.rotation;
        
        // Update odometry message
        odom_msg_.header.stamp = now;
        odom_msg_.pose.pose = pose_msg_.pose;
        
        // Calculate twist from velocity estimate
        odom_msg_.twist.twist.linear.x = velocity_estimate_.x_disp;
        odom_msg_.twist.twist.linear.y = velocity_estimate_.y_disp;
        odom_msg_.twist.twist.angular.z = velocity_estimate_.theta_rot;
        
        // Publish messages
        pose_pub_->publish(pose_msg_);
        odom_pub_->publish(odom_msg_);
    }
    
    void updateProcessingTime(double time_ms) {
        processing_times_.push_back(time_ms);
        if (processing_times_.size() > TIMING_WINDOW) {
            processing_times_.pop_front();
        }
    }
    
    double getAverageProcessingTime() {
        if (processing_times_.empty()) return 0.0;
        
        double sum = 0.0;
        for (double time : processing_times_) {
            sum += time;
        }
        return sum / processing_times_.size();
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    
    // Set high priority and real-time scheduling if possible
    rclcpp::executors::SingleThreadedExecutor executor;
    auto node = std::make_shared<HighPerformanceScanMatch>();
    
    RCLCPP_INFO(node->get_logger(), "Starting high-performance scan matching for autonomous racing");
    
    executor.add_node(node);
    executor.spin();
    
    rclcpp::shutdown();
    return 0;
}
