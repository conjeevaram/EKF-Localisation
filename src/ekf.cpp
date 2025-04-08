#include "ekf.h"
#include <iostream>
#include <random>

EKF::EKF(const Map& map, float initial_x, float initial_y) : 
    state(6),
    P(6, 6),
    Q(6, 6),
    map(map),
    landmarkNoise(0.0f, 0.1f) {
    state << initial_x, initial_y, 0.0f, 0.0f, 0.0f, 0.0f;
    P.setIdentity();
    P *= 10.0f;  // Increased initial covariance

    Q.setZero();
    Q.diagonal() << 0.1f, 0.1f, 0.1f, 0.1f, 0.1f, 0.1f;  // Adjusted process noise

    R_sensor << 2.25f, 0.0f,
                0.0f, 2.25f;
    R_landmark << 0.01f, 0.0f,
                  0.0f, 0.01f;
}

void EKF::initializeState() {
    state << 60.0f, 60.0f, 0.0f, 0.0f, 0.0f, 0.0f;
}

void EKF::predict(float dt) {
    Eigen::MatrixXf F = Eigen::MatrixXf::Identity(6, 6);
    F(0, 2) = dt;
    F(1, 3) = dt;
    state = F * state;
    P = F * P * F.transpose() + Q;
}

void EKF::updateSensor(const Eigen::Vector2f& z) {
    Eigen::MatrixXf H(2, 6);
    H << 1,0,0,0,1,0,
         0,1,0,0,0,1;

    Eigen::Vector2f y = z - H * state;
    Eigen::MatrixXf S = H * P * H.transpose() + R_sensor;
    Eigen::MatrixXf K = P * H.transpose() * S.inverse();

    state += K * y;
    P = (Eigen::MatrixXf::Identity(6,6) - K * H) * P;
}

void EKF::updateLandmark(const Eigen::Vector2f& z, float lx, float ly) {
    Eigen::MatrixXf H(2, 6);
    H << -1,0,0,0,-1,0,
          0,-1,0,0,0,-1;

    Eigen::Vector2f h(lx - state[0] - state[4], ly - state[1] - state[5]);
    Eigen::Vector2f y = z - h;

    Eigen::MatrixXf S = H * P * H.transpose() + R_landmark;
    Eigen::MatrixXf K = P * H.transpose() * S.inverse();

    state += K * y;
    P = (Eigen::MatrixXf::Identity(6,6) - K * H) * P;
}