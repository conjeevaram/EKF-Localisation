#include "ekf.h"
#include <iostream>
#include <random>
#include <cmath>

// ctor: init state vector, covariance, process noise, and sensor/landmark noise
EKF::EKF(float initial_x, float initial_y) : 
    state(4),
    P(4, 4),
    Q(4, 4),
    landmarkNoise(0.0f, 0.1f) {
    state << initial_x, initial_y, 0.0f, 0.0f;
    P.setIdentity();
    P *= 10.0f; 

    Q.setZero();
    Q.diagonal() << 0.1f, 0.1f, 0.1f, 0.1f;

    R_sensor << 2.25f, 0.0f,
                0.0f, 2.25f;
    R_landmark << 0.01f, 0.0f,
                  0.0f, 0.01f;
}

// initialize default position and velocity
void EKF::initializeState() {
    state << 60.0f, 60.0f, 0.0f, 0.0f;
}

// propagate state with constant velocity model and update covariance
void EKF::predict(float dt) {
    Eigen::MatrixXf F = Eigen::MatrixXf::Identity(4, 4);
    F(0, 2) = dt;
    F(1, 3) = dt;
    state = F * state;
    P = F * P * F.transpose() + Q;
}

// update state and covariance with sensor measurement
void EKF::updateSensor(const Eigen::Vector2f& z) {
    Eigen::MatrixXf H(2, 4);
    H << 1,0,0,0,
         0,1,0,0;

    Eigen::Vector2f y = z - H * state;
    Eigen::MatrixXf S = H * P * H.transpose() + R_sensor;
    Eigen::MatrixXf K = P * H.transpose() * S.inverse();

    state += K * y;
    P = (Eigen::MatrixXf::Identity(4,4) - K * H) * P;
}

// update state and covariance with landmark observation
void EKF::updateLandmark(const Eigen::Vector2f& z, float lx, float ly) {
    // nonlinear measurement h(x): range and bearing
    float dx = lx - state[0];
    float dy = ly - state[1];
    float range = std::sqrt(dx*dx + dy*dy);
    float bearing = std::atan2(dy, dx);
    Eigen::Vector2f h;
    h << range, bearing;

    // jacobian H
    float range_sq = dx*dx + dy*dy;
    float inv_range = 1.0f / range;
    Eigen::MatrixXf H(2,4);
    H << -dx * inv_range, -dy * inv_range, 0, 0,
          dy / range_sq,   -dx / range_sq,  0, 0;

    // innovation y
    Eigen::Vector2f y = z - h;
    // normalize bearing
    while (y[1] > M_PI)  y[1] -= 2.0f * M_PI;
    while (y[1] < -M_PI) y[1] += 2.0f * M_PI;

    Eigen::MatrixXf S = H * P * H.transpose() + R_landmark;
    Eigen::MatrixXf K = P * H.transpose() * S.inverse();

    state += K * y;
    P = (Eigen::MatrixXf::Identity(4,4) - K * H) * P;
}