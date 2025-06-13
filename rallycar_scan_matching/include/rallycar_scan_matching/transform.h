#ifndef TRANSFORM_H
#define TRANSFORM_H

#include <cmath>
#include <vector>
#include <Eigen/Dense>
#include <Eigen/Geometry>

// Forward declarations
class Point;
class Correspondence;

/**
 * Transform class representing 2D rigid body transformation
 * Optimized for real-time scan matching applications
 */
class Transform {
public:
    float x_disp;     // Translation in x
    float y_disp;     // Translation in y  
    float theta_rot;  // Rotation angle
    
    // Constructors
    Transform() : x_disp(0.0f), y_disp(0.0f), theta_rot(0.0f) {}
    
    Transform(float x, float y, float theta) 
        : x_disp(x), y_disp(y), theta_rot(theta) {}
    
    // Operators
    bool operator==(const Transform& other) const {
        const float epsilon = 1e-6f;
        return (std::abs(x_disp - other.x_disp) < epsilon) &&
               (std::abs(y_disp - other.y_disp) < epsilon) &&
               (std::abs(theta_rot - other.theta_rot) < epsilon);
    }
    
    bool operator!=(const Transform& other) const {
        return !(*this == other);
    }
    
    // Transform composition
    Transform operator+(const Transform& other) const {
        // Compose two transformations
        float cos_theta = cos(theta_rot);
        float sin_theta = sin(theta_rot);
        
        float new_x = x_disp + cos_theta * other.x_disp - sin_theta * other.y_disp;
        float new_y = y_disp + sin_theta * other.x_disp + cos_theta * other.y_disp;
        float new_theta = theta_rot + other.theta_rot;
        
        // Normalize angle
        while (new_theta > M_PI) new_theta -= 2.0f * M_PI;
        while (new_theta <= -M_PI) new_theta += 2.0f * M_PI;
        
        return Transform(new_x, new_y, new_theta);
    }
    
    // Apply transformation to a point
    void apply(const Point& input, Point& output) const;
    Point apply(const Point& input) const;
    
    // Get transformation matrix
    Eigen::Matrix3f getMatrix() const {
        Eigen::Matrix3f mat;
        float cos_theta = cos(theta_rot);
        float sin_theta = sin(theta_rot);
        
        mat << cos_theta, -sin_theta, x_disp,
               sin_theta,  cos_theta, y_disp,
               0.0f,       0.0f,      1.0f;
        
        return mat;
    }
    
    // Get inverse transformation
    Transform inverse() const {
        float cos_theta = cos(-theta_rot);
        float sin_theta = sin(-theta_rot);
        
        float inv_x = -(cos_theta * x_disp - sin_theta * y_disp);
        float inv_y = -(sin_theta * x_disp + cos_theta * y_disp);
        
        return Transform(inv_x, inv_y, -theta_rot);
    }
    
    // Reset to identity
    void reset() {
        x_disp = 0.0f;
        y_disp = 0.0f;
        theta_rot = 0.0f;
    }
    
    // Get magnitude of transformation
    float magnitude() const {
        return sqrt(x_disp * x_disp + y_disp * y_disp + theta_rot * theta_rot);
    }
};

// Function declarations

/**
 * Transform a vector of points using the given transformation
 * @param points Input points to transform
 * @param transform Transformation to apply
 * @param transformed_points Output transformed points
 */
void transformPoints(const std::vector<Point>& points, 
                    const Transform& transform, 
                    std::vector<Point>& transformed_points);

/**
 * Update transformation based on correspondences using ICP algorithm
 * This is the core function that solves for the optimal transformation
 * @param correspondences Vector of point correspondences
 * @param transform Output transformation (updated in place)
 */
void updateTransform(const std::vector<Correspondence>& correspondences, 
                    Transform& transform);

/**
 * Fast transformation update with reduced computational complexity
 * Optimized for real-time applications
 */
void updateTransformFast(const std::vector<Correspondence>& correspondences, 
                        Transform& transform);

/**
 * Weighted transformation update for better robustness
 * @param correspondences Vector of point correspondences  
 * @param weights Weights for each correspondence
 * @param transform Output transformation
 */
void updateTransformWeighted(const std::vector<Correspondence>& correspondences,
                           const std::vector<float>& weights,
                           Transform& transform);

#endif // TRANSFORM_H
