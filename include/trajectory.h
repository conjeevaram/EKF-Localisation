#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include <vector>
#include <utility>

class Trajectory {
public:
    enum TrajectoryType {
        SQUARE,
        CIRCLE
    };

    Trajectory(TrajectoryType type = CIRCLE);
    std::vector<std::pair<float, float>> getPoints() const;
    std::vector<std::pair<float, float>> getContinuousPoints(int stepsPerEdge = 100) const;
    
    // Method to switch between trajectory types
    void setTrajectoryType(TrajectoryType type);
    TrajectoryType getTrajectoryType() const { return currentType; }

private:
    std::vector<std::pair<float, float>> points;
    TrajectoryType currentType;
    
    void generateSquareTrajectory();
    void generateCircleTrajectory();
};

#endif