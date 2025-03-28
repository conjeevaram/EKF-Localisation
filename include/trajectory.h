#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include <vector>
#include <utility>

class Trajectory {
public:
    Trajectory();
    std::vector<std::pair<int, int>> getPoints() const;
    std::vector<std::pair<int, int>> getContinuousPoints(int stepsPerEdge = 100) const;

private:
    std::vector<std::pair<int, int>> points;
};

#endif