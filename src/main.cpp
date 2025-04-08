#include <SFML/Graphics.hpp>
#include <iostream>
#include <sstream>
#include <random>
#include "map.h"
#include "trajectory.h"
#include "sensorsim.h"
#include "ekf.h"

int main() {
    // ---------------------------
    // Simulation parameters
    // ---------------------------
    const unsigned int WINDOW_WIDTH = 800;
    const unsigned int WINDOW_HEIGHT = 800;
    const std::string WINDOW_TITLE = "EKF Localization";
    const float TARGET_HZ = 2000.0f;
    const float SIMULATION_DURATION = 10.0f;
    const float PCL_DENSITY = 1.0f;
    const size_t NUM_POINTS = static_cast<size_t>(TARGET_HZ * SIMULATION_DURATION * PCL_DENSITY);
    const float TIME_STEP = 1.0f / TARGET_HZ;
    const float SIMULATION_SPEED = NUM_POINTS / SIMULATION_DURATION;
    const float SCALE = 5.0f;
    const float centerX = WINDOW_WIDTH / 2.0f;
    const float centerY = WINDOW_HEIGHT / 2.0f;
    const float simCenterX = 60.0f;
    const float simCenterY = 60.0f;
    const float LANDMARK_DETECTION_RANGE = 20.0f;  // Increased from 10

    // ---------------------------
    // Window setup
    // ---------------------------
    sf::RenderWindow window(sf::VideoMode(sf::Vector2u(WINDOW_WIDTH, WINDOW_HEIGHT)), WINDOW_TITLE);
    window.setFramerateLimit(60);

    // Font setup
    sf::Font font;
    if (!font.openFromFile("assets/cmunrm.ttf")) {
        std::cerr << "Failed to load font\n";
    }

    // Text setup
    sf::Text infoText(font);
    infoText.setCharacterSize(20);
    infoText.setFillColor(sf::Color::White);
    infoText.setPosition(sf::Vector2f(10.f, 10.f));

    // ---------------------------
    // Simulation initialization
    // ---------------------------
    Map map;
    Trajectory trajectory(Trajectory::CIRCLE);
    SensorSim sensor = createSensorSim();
    auto trajectory_points = trajectory.getContinuousPoints(NUM_POINTS);
    float initial_x = trajectory_points[0].first;
    float initial_y = trajectory_points[0].second;
    EKF ekf(map, initial_x, initial_y);

    // Landmark visualization
    std::vector<sf::ConvexShape> landmarkShapes;
    for (const auto& [id, pos] : map.getLandmarks()) {
        sf::ConvexShape triangle;
        triangle.setPointCount(3);
        const float size = 3.0f;
        triangle.setPoint(0, sf::Vector2f(0.f, -size));
        triangle.setPoint(1, sf::Vector2f(-size, size));
        triangle.setPoint(2, sf::Vector2f(size, size));
        triangle.setFillColor(sf::Color::Blue);
        triangle.setPosition(sf::Vector2f(
            (pos.first - simCenterX) * SCALE + centerX,
            centerY - ((pos.second - simCenterY) * SCALE)
        ));
        landmarkShapes.push_back(triangle);
    }

    // Trajectory visualization
    sf::VertexArray groundTruthLine(sf::PrimitiveType::LineStrip);
    sf::VertexArray sensorTrail(sf::PrimitiveType::Points);
    sf::VertexArray ekfTrail(sf::PrimitiveType::Points);
    groundTruthLine.resize(trajectory_points.size());

    // Initialize ground truth trajectory
    for (size_t i = 0; i < trajectory_points.size(); ++i) {
        groundTruthLine[i].position = sf::Vector2f(
            (trajectory_points[i].first - simCenterX) * SCALE + centerX,
            centerY - ((trajectory_points[i].second - simCenterY) * SCALE)
        );
        groundTruthLine[i].color = sf::Color::Green;
    }

    // ---------------------------
    // Main simulation loop
    // ---------------------------
    float accumulatedTime = 0.0f;
    float simIndex = 0.0f;
    sf::Clock clock;
    sf::Vector2f currentSensorPos;

    while (window.isOpen()) {
        // Event handling
        while (auto event = window.pollEvent()) {
            if (event->is<sf::Event::Closed>()) {
                window.close();
            }
        }

        // Simulation update
        accumulatedTime += clock.restart().asSeconds();
        while (accumulatedTime >= TIME_STEP && simIndex < trajectory_points.size() - 1) {
            simIndex += SIMULATION_SPEED * TIME_STEP;
            simIndex = std::min(simIndex, static_cast<float>(trajectory_points.size() - 1));

            // Get ground truth position
            const int idx = static_cast<int>(simIndex);
            const auto nextIdx = (static_cast<size_t>(idx) + 1 < trajectory_points.size()) ? idx + 1 : idx;
            const float t = simIndex - idx;
            const float gt_x = trajectory_points[idx].first * (1.0f - t) + trajectory_points[nextIdx].first * t;
            const float gt_y = trajectory_points[idx].second * (1.0f - t) + trajectory_points[nextIdx].second * t;

            // Sensor update
            const auto sensorReading = sensor.update({gt_x, gt_y}, TIME_STEP);
            
            // EKF update
            ekf.predict(TIME_STEP);
            ekf.updateSensor(Eigen::Vector2f(sensorReading.first, sensorReading.second));

            std::random_device rd;
            std::mt19937 gen(rd());
            std::normal_distribution<float> landmark_noise(0.0f, 0.1f);
            
            // Inside the simulation loop:
            for (const auto& [id, pos] : map.getLandmarks()) {
                Eigen::VectorXf state = ekf.getState();
                float est_x = state[0] + state[4];
                float est_y = state[1] + state[5];
                float dx = pos.first - est_x;
                float dy = pos.second - est_y;
                if (std::hypot(dx, dy) < LANDMARK_DETECTION_RANGE) {
                    float true_z_x = pos.first - gt_x;
                    float true_z_y = pos.second - gt_y;
                    float z_x = true_z_x + landmark_noise(gen);
                    float z_y = true_z_y + landmark_noise(gen);
                    ekf.updateLandmark(Eigen::Vector2f(z_x, z_y), pos.first, pos.second);
                }
            }

            // Update sensor trail
            currentSensorPos = sf::Vector2f(
                (sensorReading.first - simCenterX) * SCALE + centerX,
                centerY - ((sensorReading.second - simCenterY) * SCALE)
            );
            sf::Vertex sensorPoint;
            sensorPoint.position = currentSensorPos;
            sensorPoint.color = sf::Color::Red;
            sensorTrail.append(sensorPoint);

            // Update EKF trail
            const Eigen::VectorXf state = ekf.getState();
            sf::Vertex ekfPoint;
            ekfPoint.position = sf::Vector2f(
                (state[0] - simCenterX) * SCALE + centerX,
                centerY - ((state[1] - simCenterY) * SCALE)
            );
            ekfPoint.color = sf::Color::Yellow;
            ekfTrail.append(ekfPoint);

            accumulatedTime -= TIME_STEP;
        }

        // ---------------------------
        // Rendering
        // ---------------------------
        window.clear(sf::Color::Black);

        // Draw landmarks
        for (const auto& shape : landmarkShapes) {
            window.draw(shape);
        }

        // Draw trajectories
        window.draw(groundTruthLine);
        window.draw(sensorTrail);
        window.draw(ekfTrail);

        // Draw current sensor position
        sf::CircleShape sensorCircle(4.f);
        sensorCircle.setFillColor(sf::Color::Red);
        sensorCircle.setOrigin(sf::Vector2f(4.f, 4.f));
        sensorCircle.setPosition(currentSensorPos);
        window.draw(sensorCircle);

        // Draw info text
        std::stringstream ss;
        ss << "Trajectories:\n"
           << "Green - Ground Truth\n"
           << "Red - Noisy Sensor\n"
           << "Yellow - EKF Estimate\n"
           << "Landmark Detection Range: " << LANDMARK_DETECTION_RANGE << " units";
        infoText.setString(ss.str());
        window.draw(infoText);

        window.display();
    }

    return 0;
}