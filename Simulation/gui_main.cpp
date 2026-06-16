#include <SFML/Graphics.hpp>
#include <iostream>
#include <vector>
#include <cmath>
#include <Eigen/Dense>
#include <optional>

#include "robot_model.hpp"
#include "inverse_kinematics.hpp"
#include "trajectory_optimiser.hpp"
#include "types.hpp"

// Configuration constants
const float SCALE = 300.0f; // Pixels per meter
const int WIDTH = 1000;
const int HEIGHT = 800;

// Helper: Convert Robot Coordinates (meters) to Screen Coordinates (pixels)
sf::Vector2f robToScr(double x, double y) {
    return sf::Vector2f(WIDTH / 2.0f + x * SCALE, HEIGHT / 2.0f - y * SCALE);
}

// Helper: Convert Screen Coordinates to Robot Coordinates (XZ Plane side-view)
// Screen X -> Robot X (Extension from base)
// Screen Y -> Robot Z (Vertical Height)
Position scrToRob(sf::Vector2i pixel, double& rx, double& rz) {
    rx = (pixel.x - WIDTH / 2.0f) / SCALE;
    rz = (HEIGHT / 2.0f - pixel.y) / SCALE;
    Position pos(3);
    pos << rx, 0.0, rz; // Solve for the arm in the XZ plane (Y=0)
    return pos;
}

int main() {
    sf::RenderWindow window(sf::VideoMode({static_cast<unsigned int>(WIDTH), static_cast<unsigned int>(HEIGHT)}), "ORION: 2D Kinematics Sim");
    window.setFramerateLimit(60);

    // Initialize Kinematics Solver
    RobotModel model;
    InverseKinematics ik;
    
    TrajectoryOptimiserParameters params;
    params.static_weight = 1.0;
    params.dynamic_weight = 0.5;
    TrajectoryOptimiser optimiser(model, params);

    JointVector current_q = JointVector::Zero(4);
    JointVector target_q = JointVector::Zero(4);
    
    double tx = 0.3, tz = 0.2;
    Position target_pos(3);
    target_pos << tx, 0.0, tz;

    sf::Font font;
    // Common font paths for Linux and macOS
    std::vector<std::string> fontPaths = {
        "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
        "/System/Library/Fonts/Supplemental/Arial.ttf",
        "/System/Library/Fonts/Helvetica.ttc",
        "arial.ttf"
    };

    bool hasFont = false;
    for (const auto& path : fontPaths) {
        if (font.openFromFile(path)) {
            hasFont = true;
            break;
        }
    }

    if (!hasFont) std::cerr << "Warning: Could not load font. UI text will be disabled.\n";

    while (window.isOpen()) {
        while (const std::optional event = window.pollEvent()) {
            if (event->is<sf::Event::Closed>()) {
                window.close();
            }
            else if (const auto* mouseButton = event->getIf<sf::Event::MouseButtonPressed>()) {
                if (mouseButton->button == sf::Mouse::Button::Left) {
                    target_pos = scrToRob(sf::Mouse::getPosition(window), tx, tz);
                    
                    // 1. Solve All IK Candidates
                    IKCandidates candidates = ik.solveAll(model, target_pos);
                    
                    if (!candidates.solutions.empty()) {
                        // 2. Prepare for Optimiser (Filtering to 4-DOF arm segments)
                        IKCandidates arm_candidates;
                        for (const auto& sol : candidates.solutions) {
                            JointVector arm(4);
                            arm << sol(0), sol(1), sol(2), sol(3);
                            arm_candidates.solutions.push_back(arm);
                        }
                        // 3. Select Best based on cost
                        target_q = optimiser.selectBest(current_q, arm_candidates);
                    }
                }
            }
        }

        // Smoothly interpolate current configuration to target
        current_q += (target_q - current_q) * 0.05;

        window.clear(sf::Color(25, 25, 25));

        // Draw Grid
        for(int i = -5; i <= 5; ++i) {
            sf::Vertex line_h[] = { {robToScr(-1, i*0.2), sf::Color(50,50,50)}, {robToScr(1, i*0.2), sf::Color(50,50,50)} };
            sf::Vertex line_v[] = { {robToScr(i*0.2, -1), sf::Color(50,50,50)}, {robToScr(i*0.2, 1), sf::Color(50,50,50)} };
            window.draw(line_h, 2, sf::PrimitiveType::Lines);
            window.draw(line_v, 2, sf::PrimitiveType::Lines);
        }

        // Visualization Logic: Draw the Arm
        // The arm starts at the shoulder mount offset defined in orion.xacro
        double base_offset_x = 0.140; // Approx shoulder_mount_x
        double base_offset_z = 0.040; // Approx shoulder_mount_z
        
        sf::Vector2f prevPoint = robToScr(base_offset_x, base_offset_z);
        double cumulative_angle = 0;
        double cur_x = base_offset_x;
        double cur_z = base_offset_z;

        for (int i = 0; i < 4; ++i) {
            cumulative_angle += current_q[i];
            double L = model.getLinkLength(i);
            
            cur_x += L * std::cos(cumulative_angle);
            cur_z += L * std::sin(cumulative_angle);
            sf::Vector2f nextPoint = robToScr(cur_x, cur_z);
            
            sf::Vertex link[] = { {prevPoint, sf::Color::Cyan}, {nextPoint, sf::Color::Cyan} };
            window.draw(link, 2, sf::PrimitiveType::Lines);
            
            sf::CircleShape joint(5.0f);
            joint.setFillColor(sf::Color::Yellow);
            joint.setOrigin({5.0f, 5.0f});
            joint.setPosition(prevPoint);
            window.draw(joint);
            
            prevPoint = nextPoint;
        }

        // Draw Target Marker
        sf::CircleShape targetMarker(8.0f);
        targetMarker.setFillColor(sf::Color::Red);
        targetMarker.setOrigin({8.0f, 8.0f});
        targetMarker.setPosition(robToScr(tx, tz));
        window.draw(targetMarker);

        // UI Text
        if (hasFont) {
            sf::Text text(font, "Left Click: Set Target | Solving via 'kinematics_solver' C++ engine", 14);
            text.setFillColor(sf::Color::White);
            text.setPosition({10.f, 10.f});
            window.draw(text);
        }

        window.display();
    }

    return 0;
}