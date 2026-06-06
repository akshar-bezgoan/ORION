#include <iostream>
#include <cmath>
#include "forward_kinematics.hpp"
 
// Helper
static void printResult(const std::string& label, const PoseMatrix& T) {
    std::cout << "\n=== " << label << " ===\n";
    std::cout << "Pose matrix:\n" << T << "\n";
    std::cout << "TCP position  x=" << T(0,3)
              << "  y=" << T(1,3) << "\n";
    std::cout << "Orientation   angle=" 
              << std::atan2(T(1,0), T(0,0)) * 180.0 / M_PI << " deg\n";
}
 
int main() {
    ForwardKinematics fk;
    RobotModel model;   // loads link lengths and joint limits from constructor
 
    // JointVector is Eigen::VectorXd — must be sized explicitly (DOF = 4)
    JointVector q(RobotModel::DOF);
 
    // Test 1: all joints at zero 
    // Arm fully extended along +X; TCP at (L1+L2+L3+L4, 0)
    q << 0.0, 0.0, 0.0, 0.0;
    printResult("All joints zero (arm fully extended)", fk.solve(model, q));
 
    // Test 2: shoulder at 90°, rest at zero
    // Arm points straight up; TCP at (0, L1+L2+L3+L4)
    q << M_PI / 2.0, 0.0, 0.0, 0.0;
    printResult("Shoulder 90 deg (arm pointing up)", fk.solve(model, q));
 
    // Test 3: representative grasp configuration 
    // shoulder=-33.7°  elbow=-0.9°  wrist=76.3°  roll=0°
    q << -33.7 * M_PI/180.0,
         -0.9  * M_PI/180.0,
          76.3 * M_PI/180.0,
          0.0;
    printResult("Grasp config (-33.7, -0.9, 76.3, 0) deg", fk.solve(model, q));
 
    // Test 4: joint limit check 
    std::cout << "\n=== Joint limit checks ===\n";
    Eigen::Vector4d q_valid(-0.5, 0.3, 1.5, 0.5);
    Eigen::Vector4d q_invalid(0.0, 0.0, 3.0, 0.0);  // wrist exceeds 2.70 rad
    std::cout << "q_valid  within limits: " 
              << (model.isWithinLimits(q_valid)   ? "YES" : "NO") << "\n";
    std::cout << "q_invalid within limits: " 
              << (model.isWithinLimits(q_invalid) ? "YES" : "NO") << "\n";
 
    return 0;
}