#include "trajectory.h"

Trajectory::Trajectory() {
    // define square trajectory
    points.push_back({10, 10});
    points.push_back({10, 110});
    points.push_back({110, 110});
    points.push_back({110, 10});
    points.push_back({10, 10});
}

std::vector<std::pair<int, int>> Trajectory::getPoints() const {
    return points;
}

std::vector<std::pair<int, int>> Trajectory::getContinuousPoints(int stepsPerEdge) const {
    std::vector<std::pair<int, int>> continuousPoints;
    if (points.empty()) {
        return continuousPoints;
    }
    
    for (size_t i = 0; i < points.size() - 1; i++) {
        auto start = points[i];
        auto end = points[i + 1];
        // interpolate points between start and end (including start, excluding end)
        for (int step = 0; step < stepsPerEdge; step++) {
            double t = static_cast<double>(step) / stepsPerEdge;
            int x = static_cast<int>(start.first + t * (end.first - start.first));
            int y = static_cast<int>(start.second + t * (end.second - start.second));
            continuousPoints.push_back({x, y});
        }
    }
    // add last point to complete path
    continuousPoints.push_back(points.back());
    return continuousPoints;
}