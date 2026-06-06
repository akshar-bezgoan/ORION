#include "robot_model.hpp"

RobotModel::RobotModel()
{
    // ── Link geometry — orion.xacro ───────────────────────────────────────
    shoulder_mount_x = 0.160;   // base_x / 2.0 = 0.320 / 2
    shoulder_h       = 0.050;
    upper_arm_l      = 0.220;
    forearm_l        = 0.145;
    gripper_l        = 0.055;
    gripper_tcp      = 0.050;

    // ── Link masses ───────────────────────────────────────────────────────
    shoulder_m  = 0.300;
    upper_arm_m = 0.308;
    forearm_m   = 0.199;
    gripper_m   = 0.120;

    // ── Joint limits ──────────────────────────────────────────────────────
    joint_limits.resize(DOF);
    joint_limits[0] = { -0.73,  1.30 };  // shoulder_joint
    joint_limits[1] = { -0.25,  0.57 };  // elbow_joint
    joint_limits[2] = {  0.00,  2.70 };  // wrist_joint
    joint_limits[3] = { -1.57,  1.57 };  // gripper_pitch_joint

    // ── Joint dynamics ────────────────────────────────────────────────────
    joint_dynamics.resize(DOF);
    joint_dynamics[0] = { 0.08, 0.10 };  // shoulder
    joint_dynamics[1] = { 0.06, 0.08 };  // elbow
    joint_dynamics[2] = { 0.04, 0.06 };  // wrist
    joint_dynamics[3] = { 0.02, 0.03 };  // gripper_pitch

    // ── Actuator limits ───────────────────────────────────────────────────
    arm_effort = 5.0;
    arm_vel    = 1.0;
}

bool RobotModel::isWithinLimits(const Eigen::Vector4d& q) const
{
    for (int i = 0; i < DOF; ++i)
        if (q[i] < joint_limits[i].min || q[i] > joint_limits[i].max)
            return false;
    return true;
}

int RobotModel::getDOF() const { return DOF; }
