#ifndef CORRESPOND_H
#define CORRESPOND_H

#include <vector>
#include <cmath>
#include <algorithm>

// Forward declarations
class Point;
class Correspondence;

/**
 * Point class for 2D laser scan points in polar coordinates
 * Optimized for high-performance scan matching
 */
class Point {
public:
    float r;     // range
    float theta; // angle
    
    Point() : r(0.0), theta(0.0) {}
    Point(float range, float angle) : r(range), theta(angle) {}
    
    // Fast distance calculation between two points
    float distToPoint2(Point* pt) const {
        float x1 = r * cos(theta);
        float y1 = r * sin(theta);
        float x2 = pt->r * cos(pt->theta);
        float y2 = pt->r * sin(pt->theta);
        
        float dx = x1 - x2;
        float dy = y1 - y2;
        return dx * dx + dy * dy; // Return squared distance for speed
    }
    
    // Convert to Cartesian coordinates
    void toCartesian(float& x, float& y) const {
        x = r * cos(theta);
        y = r * sin(theta);
    }
};

/**
 * Correspondence class for ICP algorithm
 * Links points between consecutive scans
 */
class Correspondence {
public:
    Point* p_i;      // Point in current scan
    Point* p_j;      // Point in current scan (same as p_i for scan matching)
    Point* p_i_1;    // Corresponding point in previous scan
    Point* p_i_2;    // Second best correspondence for interpolation
    
    Correspondence(Point* pi, Point* pj, Point* pi1, Point* pi2) 
        : p_i(pi), p_j(pj), p_i_1(pi1), p_i_2(pi2) {}
};

// Function declarations for correspondence algorithms

/**
 * Compute jump table for fast correspondence lookup
 * @param table Output jump table
 * @param points Input points vector
 */
void computeJump(std::vector<std::vector<int>>& table, const std::vector<Point>& points);

/**
 * Find correspondences between point sets using optimized algorithm
 * @param old_points Previous scan points
 * @param trans_points Transformed current scan points
 * @param points Original current scan points
 * @param jump_table Pre-computed jump table
 * @param c Output correspondences
 * @param prob Probability threshold for correspondence acceptance
 */
void getCorrespondence(std::vector<Point>& old_points, 
                      std::vector<Point>& trans_points, 
                      std::vector<Point>& points, 
                      std::vector<std::vector<int>>& jump_table, 
                      std::vector<Correspondence>& c, 
                      float prob);

/**
 * Fast correspondence algorithm with distance thresholding
 * Optimized for real-time performance
 */
void getCorrespondenceFast(std::vector<Point>& old_points, 
                          std::vector<Point>& trans_points, 
                          std::vector<Point>& points, 
                          std::vector<Correspondence>& c, 
                          float max_distance = 0.5);

#endif // CORRESPOND_H
