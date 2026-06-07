#pragma once

#include <Eigen/Dense>
#include <vector>

class ForwardKinematics; // forward declare so friend compiles
class InverseKinematics; // forward declare so friend compiles

class RobotModel {
public:
    RobotModel();

    static constexpr int DOF = 4;

    bool isWithinLimits(const Eigen::Vector4d& q) const;

    int getDOF() const;

private:
    // Lengths (m)
    double shoulder_h_;
    double upper_arm_l_;
    double forearm_l_;
    double gripper_l_;

    // Masses (kg)
    double shoulder_m_;
    double upper_arm_m_;
    double forearm_m_;
    double gripper_m_;

    struct JointLimit {
        double min;
        double max;
    };

    std::vector<JointLimit> joint_limits_; // size = DOF

    friend class ForwardKinematics;
    friend class InverseKinematics;
};
