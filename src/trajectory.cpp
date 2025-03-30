#include "trajectory.h"
#include <cmath>

Trajectory::Trajectory(TrajectoryType type) : currentType(type) {
    // Generate initial trajectory based on selected type
    if (type == SQUARE) {
        generateSquareTrajectory();
    } else {
        generateCircleTrajectory();
    }
}

void Trajectory::setTrajectoryType(TrajectoryType type) {
    if (type != currentType) {
        currentType = type;
        points.clear();
        
        if (type == SQUARE) {
            generateSquareTrajectory();
        } else {
            generateCircleTrajectory();
        }
    }
}

void Trajectory::generateSquareTrajectory() {
    // Define square trajectory with floating-point coordinates
    points.clear();
    points.push_back({10.0f, 10.0f});
    points.push_back({10.0f, 110.0f});
    points.push_back({110.0f, 110.0f});
    points.push_back({110.0f, 10.0f});
    points.push_back({10.0f, 10.0f}); // Close the loop
}

void Trajectory::generateCircleTrajectory() {
    // Define a circular trajectory using discrete points
    points.clear();
    const float PI = 3.14159265358979323846f;
    const float radius = 50.0f;  // Circle radius
    const float centerX = 60.0f; // Center X coordinate
    const float centerY = 60.0f; // Center Y coordinate
    
    // Use more points for a smoother circle
    const int numKeyPoints = 32;  // Increased from 8 to 32
    for (int i = 0; i < numKeyPoints; i++) {
        float angle = 2.0f * PI * i / numKeyPoints;
        float x = centerX + radius * std::cos(angle);
        float y = centerY + radius * std::sin(angle);
        points.push_back({x, y});
    }
    
    // Close the loop
    points.push_back(points[0]);
}

std::vector<std::pair<float, float>> Trajectory::getPoints() const {
    return points;
}

std::vector<std::pair<float, float>> Trajectory::getContinuousPoints(int numPoints) const {
    std::vector<std::pair<float, float>> continuousPoints;
    
    if (currentType == CIRCLE) {
        // For circle, generate points directly using parametric equation with high precision
        const float PI = 3.14159265358979323846f;
        const float radius = 50.0f;  // Circle radius
        const float centerX = 60.0f; // Center X coordinate
        const float centerY = 60.0f; // Center Y coordinate
        
        // Generate significantly more points for a smoother circle
        for (int i = 0; i < numPoints; i++) {
            float angle = 2.0f * PI * i / numPoints;
            float x = centerX + radius * std::cos(angle);
            float y = centerY + radius * std::sin(angle);
            continuousPoints.push_back({x, y});
        }
        
        // Add first point again to complete the circle
        if (!continuousPoints.empty()) {
            continuousPoints.push_back(continuousPoints[0]);
        }
    } else {
        // For square, use floating-point interpolation
        if (points.empty()) {
            return continuousPoints;
        }
        
        int pointsPerSegment = numPoints / (points.size() - 1);
        
        for (size_t i = 0; i < points.size() - 1; i++) {
            auto start = points[i];
            auto end = points[i + 1];
            // interpolate points between start and end (including start, excluding end)
            for (int step = 0; step < pointsPerSegment; step++) {
                float t = static_cast<float>(step) / pointsPerSegment;
                float x = start.first + t * (end.first - start.first);
                float y = start.second + t * (end.second - start.second);
                continuousPoints.push_back({x, y});
            }
        }
        
        // add last point to complete path
        continuousPoints.push_back(points.back());
    }
    
    return continuousPoints;
}