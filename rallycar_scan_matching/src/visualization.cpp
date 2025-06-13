#include "rallycar_scan_matching/visualization.h"
#include "rallycar_scan_matching/correspond.h"
#include "rallycar_scan_matching/transform.h"
#include <geometry_msgs/msg/point.hpp>
#include <iomanip>
#include <sstream>

// Initialize static member
int PointVisualizer::global_marker_counter_ = 0;

PointVisualizer::PointVisualizer(rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr& pub, 
                               const std::string& ns, 
                               const std::string& frame_id) 
    : publisher_(pub), namespace_(ns), frame_id_(frame_id) {
    
    clock_ = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
    marker_id_ = global_marker_counter_++;
    
    // Initialize the points marker
    points_marker_.header.frame_id = frame_id_;
    points_marker_.ns = namespace_;
    points_marker_.id = marker_id_;
    points_marker_.type = visualization_msgs::msg::Marker::POINTS;
    points_marker_.action = visualization_msgs::msg::Marker::ADD;
    points_marker_.pose.orientation.w = 1.0;
    
    // Set default scale
    points_marker_.scale.x = 0.05; // Point width
    points_marker_.scale.y = 0.05; // Point height
    
    // Reserve space for efficiency
    points_marker_.points.reserve(1000);
    points_marker_.colors.reserve(1000);
}

void PointVisualizer::addPoints(const std::vector<Point>& points, const std_msgs::msg::ColorRGBA& color) {
    for (const Point& p : points) {
        geometry_msgs::msg::Point point;
        point.x = p.r * cos(p.theta);
        point.y = p.r * sin(p.theta);
        point.z = 0.0;
        
        points_marker_.points.push_back(point);
        points_marker_.colors.push_back(color);
    }
}

void PointVisualizer::addPoint(const Point& point, const std_msgs::msg::ColorRGBA& color) {
    geometry_msgs::msg::Point p;
    p.x = point.r * cos(point.theta);
    p.y = point.r * sin(point.theta);
    p.z = 0.0;
    
    points_marker_.points.push_back(p);
    points_marker_.colors.push_back(color);
}

void PointVisualizer::clearPoints() {
    points_marker_.points.clear();
    points_marker_.colors.clear();
}

void PointVisualizer::publishPoints() {
    points_marker_.header.stamp = clock_->now();
    publisher_->publish(points_marker_);
    
    // Clear after publishing for next frame
    clearPoints();
}

void PointVisualizer::setPointScale(float scale) {
    points_marker_.scale.x = scale;
    points_marker_.scale.y = scale;
}

void PointVisualizer::setFrameId(const std::string& frame_id) {
    frame_id_ = frame_id;
    points_marker_.header.frame_id = frame_id;
}

// ScanMatchVisualizer implementation
ScanMatchVisualizer::ScanMatchVisualizer(rclcpp::Node* node, 
                                       const std::string& topic, 
                                       const std::string& frame_id)
    : node_(node), frame_id_(frame_id), marker_id_counter_(0) {
    
    publisher_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(topic, 10);
}

void ScanMatchVisualizer::visualizeCorrespondences(const std::vector<Correspondence>& correspondences) {
    visualization_msgs::msg::MarkerArray marker_array;
    
    // Create line list marker for correspondences
    visualization_msgs::msg::Marker lines;
    lines.header.frame_id = frame_id_;
    lines.header.stamp = node_->get_clock()->now();
    lines.ns = "correspondences";
    lines.id = getNextMarkerId();
    lines.type = visualization_msgs::msg::Marker::LINE_LIST;
    lines.action = visualization_msgs::msg::Marker::ADD;
    lines.pose.orientation.w = 1.0;
    
    lines.scale.x = 0.01; // Line width
    lines.color = createColor(1.0f, 1.0f, 0.0f, 0.5f); // Yellow with transparency
    
    for (const auto& corr : correspondences) {
        // Current point
        geometry_msgs::msg::Point p1;
        p1.x = corr.p_i->r * cos(corr.p_i->theta);
        p1.y = corr.p_i->r * sin(corr.p_i->theta);
        p1.z = 0.0;
        
        // Corresponding point in previous scan
        geometry_msgs::msg::Point p2;
        p2.x = corr.p_i_1->r * cos(corr.p_i_1->theta);
        p2.y = corr.p_i_1->r * sin(corr.p_i_1->theta);
        p2.z = 0.0;
        
        lines.points.push_back(p1);
        lines.points.push_back(p2);
    }
    
    marker_array.markers.push_back(lines);
    publisher_->publish(marker_array);
}

void ScanMatchVisualizer::visualizeTransform(const Transform& transform) {
    visualization_msgs::msg::MarkerArray marker_array;
    
    // Create arrow marker for transformation
    visualization_msgs::msg::Marker arrow;
    arrow.header.frame_id = frame_id_;
    arrow.header.stamp = node_->get_clock()->now();
    arrow.ns = "transform";
    arrow.id = getNextMarkerId();
    arrow.type = visualization_msgs::msg::Marker::ARROW;
    arrow.action = visualization_msgs::msg::Marker::ADD;
    
    // Position arrow at origin
    arrow.pose.position.x = 0.0;
    arrow.pose.position.y = 0.0;
    arrow.pose.position.z = 0.1; // Slightly above ground
    
    // Orient arrow according to rotation
    arrow.pose.orientation.x = 0.0;
    arrow.pose.orientation.y = 0.0;
    arrow.pose.orientation.z = sin(transform.theta_rot / 2.0);
    arrow.pose.orientation.w = cos(transform.theta_rot / 2.0);
    
    // Scale based on translation magnitude - FIXED TYPE CONVERSION
    float magnitude = sqrt(transform.x_disp * transform.x_disp + transform.y_disp * transform.y_disp);
    arrow.scale.x = std::max(0.1f, magnitude); // Length - use 0.1f instead of 0.1
    arrow.scale.y = 0.05; // Width
    arrow.scale.z = 0.05; // Height
    
    arrow.color = createColor(0.0f, 1.0f, 0.0f); // Green
    
    marker_array.markers.push_back(arrow);
    publisher_->publish(marker_array);
}

void ScanMatchVisualizer::visualizePerformanceMetrics(float frequency, float latency, int num_correspondences) {
    visualization_msgs::msg::MarkerArray marker_array;
    
    // Create text marker for performance metrics
    visualization_msgs::msg::Marker text;
    text.header.frame_id = frame_id_;
    text.header.stamp = node_->get_clock()->now();
    text.ns = "performance";
    text.id = getNextMarkerId();
    text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    text.action = visualization_msgs::msg::Marker::ADD;
    
    text.pose.position.x = 2.0;
    text.pose.position.y = 0.0;
    text.pose.position.z = 1.0;
    text.pose.orientation.w = 1.0;
    
    text.scale.z = 0.3; // Text height
    
    // Color based on performance
    if (frequency < 15.0f) {
        text.color = createColor(1.0f, 0.0f, 0.0f); // Red for poor performance
    } else if (frequency < 20.0f) {
        text.color = createColor(1.0f, 1.0f, 0.0f); // Yellow for moderate performance
    } else {
        text.color = createColor(0.0f, 1.0f, 0.0f); // Green for good performance
    }
    
    // Format performance text
    std::ostringstream oss;
    oss << "Scan Matching Performance\n";
    oss << "Frequency: " << std::fixed << std::setprecision(1) << frequency << " Hz\n";
    oss << "Latency: " << std::fixed << std::setprecision(1) << latency << " ms\n";
    oss << "Correspondences: " << num_correspondences;
    
    text.text = oss.str();
    
    marker_array.markers.push_back(text);
    publisher_->publish(marker_array);
}

void ScanMatchVisualizer::publish() {
    // This method could be used to batch publish multiple markers
    // Currently, individual methods publish immediately for real-time feedback
}

void ScanMatchVisualizer::clear() {
    visualization_msgs::msg::MarkerArray marker_array;
    
    // Create delete-all marker
    visualization_msgs::msg::Marker delete_marker;
    delete_marker.header.frame_id = frame_id_;
    delete_marker.header.stamp = node_->get_clock()->now();
    delete_marker.ns = "";
    delete_marker.id = 0;
    delete_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    
    marker_array.markers.push_back(delete_marker);
    publisher_->publish(marker_array);
}

std_msgs::msg::ColorRGBA ScanMatchVisualizer::createColor(float r, float g, float b, float a) {
    std_msgs::msg::ColorRGBA color;
    color.r = r;
    color.g = g;
    color.b = b;
    color.a = a;
    return color;
}

int ScanMatchVisualizer::getNextMarkerId() {
    return marker_id_counter_++;
}

// Color utility functions
namespace Colors {
    std_msgs::msg::ColorRGBA red(float alpha) {
        std_msgs::msg::ColorRGBA color;
        color.r = 1.0f; color.g = 0.0f; color.b = 0.0f; color.a = alpha;
        return color;
    }
    
    std_msgs::msg::ColorRGBA green(float alpha) {
        std_msgs::msg::ColorRGBA color;
        color.r = 0.0f; color.g = 1.0f; color.b = 0.0f; color.a = alpha;
        return color;
    }
    
    std_msgs::msg::ColorRGBA blue(float alpha) {
        std_msgs::msg::ColorRGBA color;
        color.r = 0.0f; color.g = 0.0f; color.b = 1.0f; color.a = alpha;
        return color;
    }
    
    std_msgs::msg::ColorRGBA yellow(float alpha) {
        std_msgs::msg::ColorRGBA color;
        color.r = 1.0f; color.g = 1.0f; color.b = 0.0f; color.a = alpha;
        return color;
    }
    
    std_msgs::msg::ColorRGBA white(float alpha) {
        std_msgs::msg::ColorRGBA color;
        color.r = 1.0f; color.g = 1.0f; color.b = 1.0f; color.a = alpha;
        return color;
    }
    
    std_msgs::msg::ColorRGBA black(float alpha) {
        std_msgs::msg::ColorRGBA color;
        color.r = 0.0f; color.g = 0.0f; color.b = 0.0f; color.a = alpha;
        return color;
    }
}
