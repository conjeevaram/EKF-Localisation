#include <SFML/Graphics.hpp>
#include "map.h"
#include "trajectory.h"
#include "sensorsim.h"

int main() {
    // ---------------------------
    // simulation & visualization parameters
    // ---------------------------
    const unsigned int WINDOW_WIDTH   = 600;
    const unsigned int WINDOW_HEIGHT  = 600;
    const unsigned int BITS_PER_PIXEL = 32;
    const std::string WINDOW_TITLE    = "Map & Sensor Visualization";

    const float TARGET_HZ           = 500.0f;       // samples per second
    const float SIMULATION_DURATION = 10.0f;        // duration of trajectory in seconds
    const float PCL_DENSITY         = 2.0f;         // increase to output a denser point cloud

    const size_t NUM_POINTS     = static_cast<size_t>(TARGET_HZ * SIMULATION_DURATION * PCL_DENSITY);
    const float TIME_STEP       = 1.0f / TARGET_HZ;
    const float SIMULATION_SPEED = NUM_POINTS / SIMULATION_DURATION; // points per second

    const float SCALE   = 5.0f;                       // scale factor for visualization
    const float Y_OFFSET = static_cast<float>(WINDOW_HEIGHT); // vertical offset

    // ---------------------------
    // window setup
    // ---------------------------
    sf::RenderWindow window(sf::VideoMode(sf::Vector2u{WINDOW_WIDTH, WINDOW_HEIGHT}, BITS_PER_PIXEL), WINDOW_TITLE);
    window.setFramerateLimit(60);

    // ---------------------------
    // simulation initialization
    // ---------------------------
    Map map;
    Trajectory trajectory;
    SensorSim sensor = createSensorSim();
    auto trajectory_points = trajectory.getContinuousPoints(NUM_POINTS);

    // ---------------------------
    // landmark visualization setup
    // ---------------------------
    std::vector<sf::CircleShape> landmarkShapes;
    for (const auto& [id, pos] : map.getLandmarks()) {
        sf::CircleShape circle(3.0f);
        circle.setFillColor(sf::Color::Red);
        circle.setOrigin(sf::Vector2f(3.0f, 3.0f));
        circle.setPosition(sf::Vector2f(
            pos.first * SCALE,
            Y_OFFSET - (pos.second * SCALE)
        ));
        landmarkShapes.push_back(circle);
    }

    // ---------------------------
    // ground truth trajectory setup
    // ---------------------------
    sf::VertexArray groundTruthLine(sf::PrimitiveType::LineStrip, trajectory_points.size());
    for (size_t i = 0; i < trajectory_points.size(); ++i) {
        groundTruthLine[i].position = sf::Vector2f(
            trajectory_points[i].first * SCALE,
            Y_OFFSET - (trajectory_points[i].second * SCALE)
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
            
            // store positions for rendering
            currentGtPos = sf::Vector2f(
                gt_x * SCALE,
                Y_OFFSET - (gt_y * SCALE)
            );
            currentSensorPos = sf::Vector2f(
                sensorReading.first * SCALE,
                Y_OFFSET - (sensorReading.second * SCALE)
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

                // draw everything
                window.clear(sf::Color::Black);
                for (const auto& shape : landmarkShapes) window.draw(shape);
                window.draw(groundTruthLine);
                window.draw(sensorTrail);
                window.draw(gtCircle);
                window.draw(sensorCircle);
                window.display();
            }
            return 0;
        }

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

        window.display();
    }

    return 0;
}
