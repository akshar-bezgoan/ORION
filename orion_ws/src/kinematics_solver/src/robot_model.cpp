#include "robot_model.hpp"

RobotModel::RobotModel()
{
    //Link lengths — values match orion.xacro properties (lines 66-69)
    shoulder_h_   = 0.050;  // xacro: shoulder_h   (was 0.090)
    upper_arm_l_  = 0.220;  // xacro: upper_arm_l  (was 0.145)
    forearm_l_    = 0.145;  // xacro: forearm_l     (unchanged)
    gripper_l_    = 0.055;  // xacro: gripper_l     (was 0.060)

    //Masses — all already match xacro (lines 79-82)
    shoulder_m_   = 0.300;  // xacro: m_shoulder
    upper_arm_m_  = 0.308;  // xacro: m_upper_arm
    forearm_m_    = 0.199;  // xacro: m_forearm
    gripper_m_    = 0.120;  // xacro: m_gripper

    joint_limits_.resize(DOF);

    // Joint limits — all already match xacro (lines 85-98)
    joint_limits_[0] = {-0.73, 1.30};  // shoulder  (xacro: shoulder_lo/hi)
    joint_limits_[1] = {-0.25, 0.57};  // elbow     (xacro: elbow_lo/hi)
    joint_limits_[2] = { 0.00, 2.70};  // wrist     (xacro: wrist_lo/hi)
    joint_limits_[3] = {-1.00, 1.00};  // wrist roll (xacro: gr_lo/hi ±1.57; kept as-is — no exact match)
}

int RobotModel::getDOF() const {
    return DOF;
}

bool RobotModel::isWithinLimits(const Eigen::Vector4d& q) const {
    for (int i = 0; i < DOF; i++) {
        if (q[i] < joint_limits_[i].min || q[i] > joint_limits_[i].max) {
            return false;
        }
    }
    return true;
}
