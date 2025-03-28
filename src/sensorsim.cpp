#include "sensorsim.h"
#include <cmath>
#include <random>

// maximum drift magnitude
constexpr float DRIFT_SATURATION = 10.0f;
// linear drift growth rate (units per second)
constexpr float DRIFT_RATE = 1.0f;
constexpr float NOISE_STRENGTH = 3.0f;

//parameters for curved drift trajectory
constexpr float CURVE_AMPLITUDE = 0.5f; // maximum angular deviation in radians (~28.6°)
constexpr float CURVE_FREQUENCY = 0.2f; // oscillation frequency (radians per second)

SensorSim::SensorSim(float driftStdDev, float measStdDev)
    : simTime(0.f),
      drift({0.f, 0.f}),
      measNoise(0.f, measStdDev),
      firstUpdate(true)
{
    std::random_device rd;
    gen.seed(rd());
    
    // compute a consistent base drift direction
    // sample two independent values from a normal distribution
    std::normal_distribution<float> driftDirDist(0.f, driftStdDev);
    float vx = driftDirDist(gen);
    float vy = driftDirDist(gen);
    float norm = std::sqrt(vx * vx + vy * vy);
    if(norm == 0.f) {
        // avoiding division by zero; set a default direction
        vx = 1.f;
        vy = 0.f;
        norm = 1.f;
    }
    baseDirection.first = vx / norm;
    baseDirection.second = vy / norm;
}

std::pair<float, float> SensorSim::update(const std::pair<float, float>& gt, float dt) {
    if(firstUpdate) {
        firstUpdate = false;
        return gt;
    }
    
    simTime += dt;
    
    // increases linearly over time, capped at DRIFT_SATURATION
    float magnitude = DRIFT_RATE * simTime;
    if(magnitude > DRIFT_SATURATION) {
        magnitude = DRIFT_SATURATION;
    }
    
    // compute an angular offset that oscillates over time
    float angleOffset = CURVE_AMPLITUDE * std::sin(CURVE_FREQUENCY * simTime);
    
    float cosA = std::cos(angleOffset);
    float sinA = std::sin(angleOffset);
    float rotatedX = baseDirection.first * cosA - baseDirection.second * sinA;
    float rotatedY = baseDirection.first * sinA + baseDirection.second * cosA;
    
    // update drift
    drift.first  = rotatedX * magnitude;
    drift.second = rotatedY * magnitude;
    
    return {
        gt.first + drift.first + measNoise(gen) * NOISE_STRENGTH,
        gt.second + drift.second + measNoise(gen) * NOISE_STRENGTH
    };
}

SensorSim createSensorSim(float driftStdDev, float measStdDev) {
    return SensorSim(driftStdDev, measStdDev);
}
