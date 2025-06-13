#include "rallycar_scan_matching/correspond.h"
#include <iostream>
#include <limits>

void computeJump(std::vector<std::vector<int>>& table, const std::vector<Point>& points) {
    const int num_points = points.size();
    
    // Clear and resize the jump table
    table.clear();
    table.resize(num_points);
    
    // For each point, compute jump indices for fast lookup
    for (int i = 0; i < num_points; ++i) {
        table[i].clear();
        table[i].reserve(20); // Reserve space for typical number of jumps
        
        // Simple jump table - store indices of nearby points
        for (int j = std::max(0, i - 10); j <= std::min(num_points - 1, i + 10); ++j) {
            if (j != i) {
                table[i].push_back(j);
            }
        }
    }
}

void getCorrespondence(std::vector<Point>& old_points, 
                      std::vector<Point>& trans_points, 
                      std::vector<Point>& points, 
                      std::vector<std::vector<int>>& /* jump_table */, // Mark as unused
                      std::vector<Correspondence>& c, 
                      float /* prob */) { // Mark as unused
    
    c.clear();
    c.reserve(trans_points.size());
    
    const int trans_size = trans_points.size();
    const int old_size = old_points.size();
    
    // For each transformed point, find best correspondence
    for (int i = 0; i < trans_size; ++i) {
        int best_idx = 0;
        int second_best_idx = 1;
        float best_dist = std::numeric_limits<float>::max();
        float second_best_dist = std::numeric_limits<float>::max();
        
        // Search window around expected position
        int search_start = std::max(0, i - 20);
        int search_end = std::min(old_size, i + 21);
        
        for (int j = search_start; j < search_end; ++j) {
            float dist = trans_points[i].distToPoint2(&old_points[j]);
            
            if (dist < best_dist) {
                second_best_dist = best_dist;
                second_best_idx = best_idx;
                best_dist = dist;
                best_idx = j;
            } else if (dist < second_best_dist) {
                second_best_dist = dist;
                second_best_idx = j;
            }
        }
        
        // Only add correspondence if distance is reasonable
        if (best_dist < 0.25) { // 0.5m threshold squared
            c.emplace_back(&trans_points[i], &points[i], 
                          &old_points[best_idx], &old_points[second_best_idx]);
        }
    }
}

void getCorrespondenceFast(std::vector<Point>& old_points, 
                          std::vector<Point>& trans_points, 
                          std::vector<Point>& points, 
                          std::vector<Correspondence>& c, 
                          float max_distance) {
    
    c.clear();
    c.reserve(trans_points.size() / 2); // Estimate fewer correspondences for speed
    
    const int trans_size = trans_points.size();
    const int old_size = old_points.size();
    const float max_dist_sq = max_distance * max_distance;
    
    // Fast correspondence with limited search
    for (int i = 0; i < trans_size; ++i) {
        int best_idx = -1;
        int second_best_idx = -1;
        float best_dist = max_dist_sq;
        
        // Limited search window for speed
        int search_radius = 15; // Reduced search radius
        int search_start = std::max(0, i - search_radius);
        int search_end = std::min(old_size, i + search_radius + 1);
        
        for (int j = search_start; j < search_end; ++j) {
            float dist = trans_points[i].distToPoint2(&old_points[j]);
            
            if (dist < best_dist) {
                best_dist = dist;
                best_idx = j;
            }
        }
        
        // Find second best for interpolation
        if (best_idx != -1) {
            second_best_idx = (best_idx > 0) ? best_idx - 1 : best_idx + 1;
            if (second_best_idx >= old_size) {
                second_best_idx = old_size - 1;
            }
            
            c.emplace_back(&trans_points[i], &points[i], 
                          &old_points[best_idx], &old_points[second_best_idx]);
        }
    }
}
