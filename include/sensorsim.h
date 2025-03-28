#ifndef SENSORSIM_H
#define SENSORSIM_H

#include <utility>
#include <random>

class SensorSim {
public:
    SensorSim(float driftStdDev, float measStdDev);
    std::pair<float, float> update(const std::pair<float, float>& gt, float dt);
    std::pair<float, float> getAcceleration() const { return accel; }

private:
    float simTime;
    std::pair<float, float> drift;
    std::pair<float, float> baseDirection;
    std::pair<float, float> accel;
    std::normal_distribution<float> measNoise;
    std::default_random_engine gen;
    bool firstUpdate;
};

SensorSim createSensorSim(float driftStdDev = 1.0f, float measStdDev = 1.0f);

#endif