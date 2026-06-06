#pragma once

#include <Eigen/Dense>
#include <vector>

class RobotModel {
public:
    RobotModel();

    static constexpr int DOF = 4;

    Eigen::Matrix4d forwardKinematics(const Eigen::Vector4d& q) const;

    bool isWithinLimits(const Eigen::Vector4d& q) const;

    int getDOF() const;

private:
    // Lengths (m)
    double shoulder_h_;
    double upper_arm_l_;
    double forearm_l_;
    double hand_l_;

    // Masses (kg)
    double shoulder_m_;
    double upper_arm_m_;
    double forearm_m_;
    double hand_m_;

    struct JointLimit {
        double min;
        double max;
    };

    std::vector<JointLimit> joint_limits_; // size = DOF
};