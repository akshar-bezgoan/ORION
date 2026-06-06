#pragma once

#include <Eigen/Dense>
#include <vector>

// ---------------------------------------------------------------------------
//  RobotModel — physical constants of the Orion arm, sourced from orion.xacro
//  No logic, no computation. Pure ground truth.
// ---------------------------------------------------------------------------

class RobotModel {
public:
    RobotModel();

    static constexpr int DOF = 4;

    // ── Link geometry (metres) ────────────────────────────────────────────
    double shoulder_mount_x;  // shoulder joint X in base frame       = 0.160
    double shoulder_h;        // shoulder stub height (+Z)             = 0.050
    double upper_arm_l;       // upper arm length (+X after elbow)     = 0.220
    double forearm_l;         // forearm length (+X after wrist)       = 0.145
    double gripper_l;         // gripper base depth (+X)               = 0.055
    double gripper_tcp;       // TCP offset past gripper base (+X)     = 0.050

    // ── Link masses (kg) ──────────────────────────────────────────────────
    double shoulder_m;        // 0.300
    double upper_arm_m;       // 0.308
    double forearm_m;         // 0.199
    double gripper_m;         // 0.120

    // ── Joint limits (rad) ────────────────────────────────────────────────
    struct JointLimit {
        double min;
        double max;
    };

    std::vector<JointLimit> joint_limits;  // size = DOF
    //   [0] shoulder_joint   [-0.73,  1.30]
    //   [1] elbow_joint      [-0.25,  0.57]
    //   [2] wrist_joint      [ 0.00,  2.70]
    //   [3] gripper_pitch    [-1.57,  1.57]

    // ── Joint dynamics ────────────────────────────────────────────────────
    struct JointDynamics {
        double damping;
        double friction;
    };

    std::vector<JointDynamics> joint_dynamics;  // size = DOF
    //   [0] shoulder   damping=0.08  friction=0.10
    //   [1] elbow      damping=0.06  friction=0.08
    //   [2] wrist      damping=0.04  friction=0.06
    //   [3] gripper    damping=0.02  friction=0.03

    // ── Actuator limits ───────────────────────────────────────────────────
    double arm_effort;        // 5.0 Nm  — all arm joints
    double arm_vel;           // 1.0 rad/s

    // ── Convenience ───────────────────────────────────────────────────────
    bool isWithinLimits(const Eigen::Vector4d& q) const;
    int  getDOF() const;
};
