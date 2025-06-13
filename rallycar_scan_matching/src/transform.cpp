#include "rallycar_scan_matching/transform.h"
#include "rallycar_scan_matching/correspond.h"
#include <iostream>
#include <cmath>

void Transform::apply(const Point& input, Point& output) const {
    // Convert to Cartesian
    float x = input.r * cos(input.theta);
    float y = input.r * sin(input.theta);
    
    // Apply transformation
    float cos_theta = cos(theta_rot);
    float sin_theta = sin(theta_rot);
    
    float new_x = cos_theta * x - sin_theta * y + x_disp;
    float new_y = sin_theta * x + cos_theta * y + y_disp;
    
    // Convert back to polar
    output.r = sqrt(new_x * new_x + new_y * new_y);
    output.theta = atan2(new_y, new_x);
}

Point Transform::apply(const Point& input) const {
    Point output;
    apply(input, output);
    return output;
}

void transformPoints(const std::vector<Point>& points, 
                    const Transform& transform, 
                    std::vector<Point>& transformed_points) {
    
    transformed_points.clear();
    transformed_points.reserve(points.size());
    
    // Pre-compute trigonometric values for efficiency
    float cos_theta = cos(transform.theta_rot);
    float sin_theta = sin(transform.theta_rot);
    
    for (const Point& p : points) {
        // Convert to Cartesian
        float x = p.r * cos(p.theta);
        float y = p.r * sin(p.theta);
        
        // Apply transformation
        float new_x = cos_theta * x - sin_theta * y + transform.x_disp;
        float new_y = sin_theta * x + cos_theta * y + transform.y_disp;
        
        // Convert back to polar and add to output
        transformed_points.emplace_back(
            sqrt(new_x * new_x + new_y * new_y),
            atan2(new_y, new_x)
        );
    }
}

void updateTransform(const std::vector<Correspondence>& correspondences, 
                    Transform& transform) {
    
    if (correspondences.empty()) {
        return;
    }
    
    const int num_corr = correspondences.size();
    
    // Use simplified ICP approach for real-time performance
    // Compute centroids
    float cx1 = 0.0f, cy1 = 0.0f; // Current scan centroid
    float cx2 = 0.0f, cy2 = 0.0f; // Previous scan centroid
    
    for (const auto& corr : correspondences) {
        float x1, y1, x2, y2;
        corr.p_i->toCartesian(x1, y1);
        corr.p_i_1->toCartesian(x2, y2);
        
        cx1 += x1;
        cy1 += y1;
        cx2 += x2;
        cy2 += y2;
    }
    
    cx1 /= num_corr;
    cy1 /= num_corr;
    cx2 /= num_corr;
    cy2 /= num_corr;
    
    // Compute cross-covariance matrix
    float Sxx = 0.0f, Sxy = 0.0f, Syx = 0.0f, Syy = 0.0f;
    
    for (const auto& corr : correspondences) {
        float x1, y1, x2, y2;
        corr.p_i->toCartesian(x1, y1);
        corr.p_i_1->toCartesian(x2, y2);
        
        float dx1 = x1 - cx1;
        float dy1 = y1 - cy1;
        float dx2 = x2 - cx2;
        float dy2 = y2 - cy2;
        
        Sxx += dx1 * dx2;
        Sxy += dx1 * dy2;
        Syx += dy1 * dx2;
        Syy += dy1 * dy2;
    }
    
    // Compute rotation using SVD approximation
    float angle = atan2(Sxy - Syx, Sxx + Syy);
    
    // Compute translation
    float cos_angle = cos(angle);
    float sin_angle = sin(angle);
    
    float tx = cx2 - (cos_angle * cx1 - sin_angle * cy1);
    float ty = cy2 - (sin_angle * cx1 + cos_angle * cy1);
    
    // Update transform
    transform.x_disp = tx;
    transform.y_disp = ty;
    transform.theta_rot = angle;
    
    // Normalize angle
    while (transform.theta_rot > M_PI) {
        transform.theta_rot -= 2.0f * M_PI;
    }
    while (transform.theta_rot <= -M_PI) {
        transform.theta_rot += 2.0f * M_PI;
    }
}

void updateTransformFast(const std::vector<Correspondence>& correspondences, 
                        Transform& transform) {
    
    if (correspondences.empty()) {
        return;
    }
    
    // Use weighted least squares for speed
    float sum_weight = 0.0f;
    float sum_dx = 0.0f, sum_dy = 0.0f;
    float sum_cross = 0.0f, sum_dot = 0.0f;
    
    for (const auto& corr : correspondences) {
        float x1, y1, x2, y2;
        corr.p_i->toCartesian(x1, y1);
        corr.p_i_1->toCartesian(x2, y2);
        
        float weight = 1.0f; // Could be distance-based
        
        sum_weight += weight;
        sum_dx += weight * (x2 - x1);
        sum_dy += weight * (y2 - y1);
        
        // For rotation estimation
        sum_cross += weight * (x1 * y2 - y1 * x2);
        sum_dot += weight * (x1 * x2 + y1 * y2);
    }
    
    if (sum_weight > 0.0f) {
        // Translation estimate
        transform.x_disp = sum_dx / sum_weight;
        transform.y_disp = sum_dy / sum_weight;
        
        // Rotation estimate
        transform.theta_rot = atan2(sum_cross, sum_dot);
        
        // Normalize angle
        while (transform.theta_rot > M_PI) {
            transform.theta_rot -= 2.0f * M_PI;
        }
        while (transform.theta_rot <= -M_PI) {
            transform.theta_rot += 2.0f * M_PI;
        }
        
        // Scale down for incremental updates
        const float alpha = 0.8f; // Damping factor
        transform.x_disp *= alpha;
        transform.y_disp *= alpha;
        transform.theta_rot *= alpha;
    }
}

void updateTransformWeighted(const std::vector<Correspondence>& correspondences,
                           const std::vector<float>& weights,
                           Transform& transform) {
    
    if (correspondences.empty() || weights.size() != correspondences.size()) {
        return;
    }
    
    // Weighted centroid computation
    float total_weight = 0.0f;
    float cx1 = 0.0f, cy1 = 0.0f;
    float cx2 = 0.0f, cy2 = 0.0f;
    
    for (size_t i = 0; i < correspondences.size(); ++i) {
        float x1, y1, x2, y2;
        correspondences[i].p_i->toCartesian(x1, y1);
        correspondences[i].p_i_1->toCartesian(x2, y2);
        
        float w = weights[i];
        total_weight += w;
        
        cx1 += w * x1;
        cy1 += w * y1;
        cx2 += w * x2;
        cy2 += w * y2;
    }
    
    if (total_weight > 0.0f) {
        cx1 /= total_weight;
        cy1 /= total_weight;
        cx2 /= total_weight;
        cy2 /= total_weight;
        
        // Weighted covariance computation
        float Sxx = 0.0f, Sxy = 0.0f, Syx = 0.0f, Syy = 0.0f;
        
        for (size_t i = 0; i < correspondences.size(); ++i) {
            float x1, y1, x2, y2;
            correspondences[i].p_i->toCartesian(x1, y1);
            correspondences[i].p_i_1->toCartesian(x2, y2);
            
            float w = weights[i];
            float dx1 = x1 - cx1;
            float dy1 = y1 - cy1;
            float dx2 = x2 - cx2;
            float dy2 = y2 - cy2;
            
            Sxx += w * dx1 * dx2;
            Sxy += w * dx1 * dy2;
            Syx += w * dy1 * dx2;
            Syy += w * dy1 * dy2;
        }
        
        // Compute optimal transformation
        float angle = atan2(Sxy - Syx, Sxx + Syy);
        
        float cos_angle = cos(angle);
        float sin_angle = sin(angle);
        
        float tx = cx2 - (cos_angle * cx1 - sin_angle * cy1);
        float ty = cy2 - (sin_angle * cx1 + cos_angle * cy1);
        
        transform.x_disp = tx;
        transform.y_disp = ty;
        transform.theta_rot = angle;
        
        // Normalize angle
        while (transform.theta_rot > M_PI) {
            transform.theta_rot -= 2.0f * M_PI;
        }
        while (transform.theta_rot <= -M_PI) {
            transform.theta_rot += 2.0f * M_PI;
        }
    }
}
