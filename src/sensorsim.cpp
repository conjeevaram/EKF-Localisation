#include "sensorsim.h"
#include <cmath>
#include <random>

constexpr float DRIFT_SATURATION = 15.0f;
constexpr float DRIFT_RATE = 1.0f;
constexpr float NOISE_STRENGTH = 1.5f;
constexpr float CURVE_AMPLITUDE = 0.7f;
constexpr float CURVE_FREQUENCY = 0.2f;

SensorSim::SensorSim(float driftStdDev, float measStdDev) :
    simTime(0.f),
    drift({0.f, 0.f}),
    accel({0.f, 0.f}),
    measNoise(0.f, measStdDev),
    firstUpdate(true)
{
    std::random_device rd;
    gen.seed(rd());
    
    // base drift direction from normal distribution sample
    std::normal_distribution<float> driftDirDist(0.f, driftStdDev);
    float vx = driftDirDist(gen);
    float vy = driftDirDist(gen);
    float norm = std::sqrt(vx*vx + vy*vy);
    
    if(norm == 0.f) {
        vx = 1.f;
        vy = 0.f;
        norm = 1.f;
    }
    baseDirection = {vx/norm, vy/norm};
}

std::pair<float, float> SensorSim::update(const std::pair<float, float>& gt, float dt) {
    if(firstUpdate) {
        firstUpdate = false;
        return gt;
    }
    
    simTime += dt;
    
    float magnitude = DRIFT_RATE * simTime;
    if(magnitude > DRIFT_SATURATION) magnitude = DRIFT_SATURATION;
    
    float theta = CURVE_AMPLITUDE * std::sin(CURVE_FREQUENCY * simTime);
    float theta_dot = CURVE_AMPLITUDE * CURVE_FREQUENCY * std::cos(CURVE_FREQUENCY * simTime);
    float theta_ddot = -CURVE_AMPLITUDE * CURVE_FREQUENCY * CURVE_FREQUENCY * std::sin(CURVE_FREQUENCY * simTime);
    float dm_dt = (magnitude < DRIFT_SATURATION) ? DRIFT_RATE : 0.0f;

    float cosA = std::cos(theta);
    float sinA = std::sin(theta);
    float rotatedX = baseDirection.first * cosA - baseDirection.second * sinA;
    float rotatedY = baseDirection.first * sinA + baseDirection.second * cosA;

    // update drift
    drift.first  = rotatedX * magnitude;
    drift.second = rotatedY * magnitude;

    // compute acceleration components
    float term1_x = 2.0f * dm_dt * (-theta_dot * rotatedY);
    float term2_x = magnitude * (-theta_ddot * rotatedY - theta_dot * theta_dot * rotatedX);
    accel.first = term1_x + term2_x;

    float term1_y = 2.0f * dm_dt * (theta_dot * rotatedX);
    float term2_y = magnitude * (theta_ddot * rotatedX - theta_dot * theta_dot * rotatedY);
    accel.second = term1_y + term2_y;

    // apply measurement noise to position
    return {
        gt.first + drift.first + measNoise(gen) * NOISE_STRENGTH,
        gt.second + drift.second + measNoise(gen) * NOISE_STRENGTH
    };
}

SensorSim createSensorSim(float driftStdDev, float measStdDev) {
    return SensorSim(driftStdDev, measStdDev);
}