#ifndef EKF_H
#define EKF_H

#include <Eigen/Dense>
#include "map.h"
#include <random>

class EKF {
public:
    EKF(const Map& map, float initial_x, float initial_y);

    void predict(float dt);
    void updateSensor(const Eigen::Vector2f& z);
    void updateLandmark(const Eigen::Vector2f& z, float lx, float ly);

    Eigen::VectorXf getState() const { return state; }
    Eigen::MatrixXf getCovariance() const { return P; }

private:
    Eigen::VectorXf state; // [x, y, vx, vy]
    Eigen::MatrixXf P;     // Covariance matrix
    Eigen::MatrixXf Q;     // Process noise covariance
    Eigen::Matrix2f R_sensor, R_landmark;
    const Map& map;
    std::default_random_engine gen;
    std::normal_distribution<float> landmarkNoise;

    void initializeState();
};

#endif