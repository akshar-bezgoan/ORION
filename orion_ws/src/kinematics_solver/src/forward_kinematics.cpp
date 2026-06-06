#include <iostream>
#include <cmath>
#include "forward_kinematics.hpp"

PoseMatrix ForwardKinematics::solve(const RobotModel& model, const JointVector& q) const {

    // Link lengths  (L1 = shoulder_h_, L2 = upper_arm_l_, ...)
    const double L1 = model.shoulder_h_;
    const double L2 = model.upper_arm_l_;
    const double L3 = model.forearm_l_;
    const double L4 = model.gripper_l_;

    // Cumulative angles
    const double t1   = q(0);
    const double t12  = q(0) + q(1);
    const double t123 = q(0) + q(1) + q(2);
    const double S    = q(0) + q(1) + q(2) + q(3);   // total angle

    // End-effector position
    const double px = L1*std::cos(t1) + L2*std::cos(t12) + L3*std::cos(t123) + L4*std::cos(S);
    const double py = L1*std::sin(t1) + L2*std::sin(t12) + L3*std::sin(t123) + L4*std::sin(S);

    PoseMatrix EndPosition;
    EndPosition <<
        std::cos(S), -std::sin(S),  0,  px,
        std::sin(S),  std::cos(S),  0,  py,
        0,            0,            1,  0,
        0,            0,            0,  1;

    return EndPosition;
}
