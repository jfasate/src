#ifndef VISUALIZATION_H
#define VISUALIZATION_H

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <vector>
#include <string>

// Forward declaration
class Point;

/**
 * PointVisualizer class for visualizing scan matching data in RViz
 * Optimized for high-frequency LiDAR data visualization
 */
class PointVisualizer {
private:
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr& publisher_;
    visualization_msgs::msg::Marker points_marker_;
    rclcpp::Clock::SharedPtr clock_;
    std::string namespace_;
    std::string frame_id_;
    int marker_id_;
    
    static int global_marker_counter_;

public:
    /**
     * Constructor
     * @param pub Reference to RViz marker publisher
     * @param ns Namespace for markers
     * @param frame_id Frame ID for visualization
     */
    PointVisualizer(rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr& pub, 
                   const std::string& ns, 
                   const std::string& frame_id);
    
    /**
     * Destructor
     */
    ~PointVisualizer() = default;
    
    /**
     * Add points to the visualization with specified color
     * @param points Vector of points to visualize
     * @param color Color for the points
     */
    void addPoints(const std::vector<Point>& points, const std_msgs::msg::ColorRGBA& color);
    
    /**
     * Add a single point to the visualization
     * @param point Point to add
     * @param color Color for the point
     */
    void addPoint(const Point& point, const std_msgs::msg::ColorRGBA& color);
    
    /**
     * Clear all points from the visualization
     */
    void clearPoints();
    
    /**
     * Publish the accumulated points to RViz
     */
    void publishPoints();
    
    /**
     * Set the scale of the point markers
     * @param scale Scale factor for point size
     */
    void setPointScale(float scale);
    
    /**
     * Set the frame ID for the visualization
     * @param frame_id New frame ID
     */
    void setFrameId(const std::string& frame_id);
};

/**
 * ScanMatchVisualizer class for comprehensive scan matching visualization
 * Provides multiple visualization modes for debugging and analysis
 */
class ScanMatchVisualizer {
private:
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher_;
    rclcpp::Node* node_;
    std::string frame_id_;
    int marker_id_counter_;

public:
    /**
     * Constructor
     * @param node Pointer to ROS2 node
     * @param topic Topic name for marker array publisher
     * @param frame_id Frame ID for visualization
     */
    ScanMatchVisualizer(rclcpp::Node* node, 
                       const std::string& topic, 
                       const std::string& frame_id);
    
    /**
     * Visualize correspondences between point sets
     * @param correspondences Vector of point correspondences
     */
    void visualizeCorrespondences(const std::vector<class Correspondence>& correspondences);
    
    /**
     * Visualize transformation arrow
     * @param transform Transformation to visualize
     */
    void visualizeTransform(const class Transform& transform);
    
    /**
     * Visualize performance metrics as text
     * @param frequency Processing frequency
     * @param latency Processing latency in ms
     * @param num_correspondences Number of correspondences
     */
    void visualizePerformanceMetrics(float frequency, float latency, int num_correspondences);
    
    /**
     * Publish all accumulated visualizations
     */
    void publish();
    
    /**
     * Clear all visualizations
     */
    void clear();

private:
    /**
     * Create a standard color for different visualization types
     */
    std_msgs::msg::ColorRGBA createColor(float r, float g, float b, float a = 1.0f);
    
    /**
     * Get next unique marker ID
     */
    int getNextMarkerId();
};

// Utility functions for creating common colors
namespace Colors {
    std_msgs::msg::ColorRGBA red(float alpha = 1.0f);
    std_msgs::msg::ColorRGBA green(float alpha = 1.0f);
    std_msgs::msg::ColorRGBA blue(float alpha = 1.0f);
    std_msgs::msg::ColorRGBA yellow(float alpha = 1.0f);
    std_msgs::msg::ColorRGBA white(float alpha = 1.0f);
    std_msgs::msg::ColorRGBA black(float alpha = 1.0f);
}

#endif // VISUALIZATION_H
