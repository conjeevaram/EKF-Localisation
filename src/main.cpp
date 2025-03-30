#include <SFML/Graphics.hpp>
#include <iostream>
#include "map.h"
#include "trajectory.h"
#include "sensorsim.h"
#include <sstream>  // for string formatting

int main() {
    // ---------------------------
    // simulation & visualization parameters
    // ---------------------------
    const unsigned int WINDOW_WIDTH   = 800; // updated for larger viewing area
    const unsigned int WINDOW_HEIGHT  = 800; // updated for larger viewing area
    const unsigned int BITS_PER_PIXEL = 32;
    const std::string WINDOW_TITLE    = "Map & Sensor Visualization";

    const float TARGET_HZ           = 1000.0f;       // samples per second
    const float SIMULATION_DURATION = 10.0f;        // duration of trajectory in seconds
    const float PCL_DENSITY         = 1.0f;         // increase to output a denser point cloud

    const size_t NUM_POINTS     = static_cast<size_t>(TARGET_HZ * SIMULATION_DURATION * PCL_DENSITY);
    const float TIME_STEP       = 1.0f / TARGET_HZ;
    const float SIMULATION_SPEED = NUM_POINTS / SIMULATION_DURATION; // points per second

    const float SCALE   = 5.0f;                       // scale factor for visualization
    const float centerX = WINDOW_WIDTH / 2.0f;        // new center X coordinate
    const float centerY = WINDOW_HEIGHT / 2.0f;       // new center Y coordinate
    const float simCenterX = 60.0f;                   // simulation center X
    const float simCenterY = 60.0f;                   // simulation center Y

    // ---------------------------
    // window setup
    // ---------------------------
    sf::RenderWindow window(sf::VideoMode(sf::Vector2u{WINDOW_WIDTH, WINDOW_HEIGHT}, BITS_PER_PIXEL), WINDOW_TITLE);
    window.setFramerateLimit(60);

    // Setup for text rendering (acceleration display)
    sf::Font font;
    if (!font.openFromFile("/Users/paras/Projects/EKF-Localisation/assets/cmunrm.ttf")) {
        std::cerr << "Failed to load font\n";
    }
    
    sf::Text accelText(font);
    
    accelText.setString("Acceleration: ");
    accelText.setCharacterSize(24);
    accelText.setFillColor(sf::Color::White);
    accelText.setPosition(sf::Vector2f(10.f, 10.f));  // Position in top-left corner

    // ---------------------------
    // simulation initialization
    // ---------------------------
    Map map;
    
    // Create trajectory with desired type (CIRCLE is default)
    Trajectory trajectory(Trajectory::CIRCLE);  // or Trajectory::SQUARE
    
    // To switch trajectory type:
    // trajectory.setTrajectoryType(Trajectory::SQUARE);
    
    SensorSim sensor = createSensorSim();
    auto trajectory_points = trajectory.getContinuousPoints(NUM_POINTS);

    // ---------------------------
    // landmark visualization setup
    // ---------------------------
    std::vector<sf::ConvexShape> landmarkShapes;
    for (const auto& [id, pos] : map.getLandmarks()) {
        sf::ConvexShape triangle;
        triangle.setPointCount(3);
        float size = 3.0f;
        // Define an equilateral triangle pointing upward
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

    // ---------------------------
    // ground truth trajectory setup
    // ---------------------------
    sf::VertexArray groundTruthLine(sf::PrimitiveType::LineStrip, trajectory_points.size());
    for (size_t i = 0; i < trajectory_points.size(); ++i) {
        groundTruthLine[i].position = sf::Vector2f(
            (trajectory_points[i].first - simCenterX) * SCALE + centerX,
            centerY - ((trajectory_points[i].second - simCenterY) * SCALE)
        );
        groundTruthLine[i].color = sf::Color::Green;
    }

    // ---------------------------
    // sensor data storage setup
    // ---------------------------
    sf::VertexArray sensorTrail(sf::PrimitiveType::Points);
    float accumulatedTime = 0.0f;
    float simIndex = 0.0f;
    sf::Vector2f currentSensorPos;
    sf::Vector2f currentGtPos;

    // ---------------------------
    // main simulation loop
    // ---------------------------
    sf::Clock clock;
    while (window.isOpen()) {
        // poll events using the provided event loop style
        while (auto event = window.pollEvent()) {
            if (event->is<sf::Event::Closed>()) {
                window.close();
            }
        }

        // fixed-step simulation updates
        accumulatedTime += clock.restart().asSeconds();
        while (accumulatedTime >= TIME_STEP && simIndex < trajectory_points.size() - 1) {
            // update simulation index
            simIndex += SIMULATION_SPEED * TIME_STEP;
            simIndex = std::min(simIndex, static_cast<float>(trajectory_points.size() - 1));

            // calculate ground truth position
            int idx = static_cast<int>(simIndex);
            int nextIdx = (idx + 1 < trajectory_points.size()) ? idx + 1 : idx;
            float t = simIndex - idx;
            float gt_x = trajectory_points[idx].first * (1.0f - t) + trajectory_points[nextIdx].first * t;
            float gt_y = trajectory_points[idx].second * (1.0f - t) + trajectory_points[nextIdx].second * t;

            // get sensor reading
            std::pair<float, float> sensorReading = sensor.update({gt_x, gt_y}, TIME_STEP);
            
            // update positions using centered coordinate system
            currentGtPos = sf::Vector2f(
                (gt_x - simCenterX) * SCALE + centerX,
                centerY - ((gt_y - simCenterY) * SCALE)
            );
            currentSensorPos = sf::Vector2f(
                (sensorReading.first - simCenterX) * SCALE + centerX,
                centerY - ((sensorReading.second - simCenterY) * SCALE)
            );

            // add to point cloud
            sf::Vertex vertex;
            vertex.position = currentSensorPos;
            vertex.color = sf::Color::Red;
            sensorTrail.append(vertex);

            accumulatedTime -= TIME_STEP;
        }

        // check for trajectory completion
        if (simIndex >= trajectory_points.size() - 1) {
            // freeze final frame
            sf::CircleShape gtCircle(4.f), sensorCircle(4.f);
            gtCircle.setFillColor(sf::Color::Green);
            sensorCircle.setFillColor(sf::Color::Red);
            gtCircle.setOrigin(sf::Vector2f(4.f, 4.f));
            sensorCircle.setOrigin(sf::Vector2f(4.f, 4.f));
            gtCircle.setPosition(currentGtPos);
            sensorCircle.setPosition(currentSensorPos);

            while (window.isOpen()) {
                while (auto event = window.pollEvent()) {
                    if (event->is<sf::Event::Closed>()) {
                        window.close();
                    }
                }

                // get acceleration for display
                auto accel = sensor.getAcceleration();
                std::stringstream ss;
                ss << "Acceleration: (" << std::fixed << std::setprecision(2) 
                   << accel.first << ", " << accel.second << ")";
                accelText.setString(ss.str());

                // draw everything
                window.clear(sf::Color::Black);
                for (const auto& shape : landmarkShapes) window.draw(shape);
                window.draw(groundTruthLine);
                window.draw(sensorTrail);
                window.draw(gtCircle);
                window.draw(sensorCircle);
                window.draw(accelText);
                window.display();
            }
            return 0;
        }

        // Get acceleration for display
        auto accel = sensor.getAcceleration();
        std::stringstream ss;
        ss << "Acceleration: (" << std::fixed << std::setprecision(2) 
           << accel.first << ", " << accel.second << ")";
        accelText.setString(ss.str());

        // render frame
        window.clear(sf::Color::Black);
        
        // draw landmarks
        for (const auto& shape : landmarkShapes) {
            window.draw(shape);
        }

        // draw trajectories
        window.draw(groundTruthLine);
        window.draw(sensorTrail);

        // draw current positions
        sf::CircleShape gtCircle(4.f), sensorCircle(4.f);
        gtCircle.setFillColor(sf::Color::Green);
        sensorCircle.setFillColor(sf::Color::Red);
        gtCircle.setOrigin(sf::Vector2f(4.f, 4.f));
        sensorCircle.setOrigin(sf::Vector2f(4.f, 4.f));
        gtCircle.setPosition(currentGtPos);
        sensorCircle.setPosition(currentSensorPos);
        window.draw(gtCircle);
        window.draw(sensorCircle);
        window.draw(accelText);

        window.display();
    }

    return 0;
}