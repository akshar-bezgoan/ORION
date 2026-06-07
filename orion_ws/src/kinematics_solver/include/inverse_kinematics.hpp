#pragma once
#include "robot_model.hpp"
#include "types.hpp"


class InverseKinematics {
public:
    IKCandidates solveAll(const RobotModel& model,
                          const Position&   target) const;

    JointVector solve(RobotModel& model, Position end_position) const;

    bool check(RobotModel& model, Position end_position,
               JointVector& q) const;

    static constexpr double kPosTol   = 1.0e-4;  // metres — FK round-trip tolerance
    static constexpr double kJointTol = 1.0e-4;  // radians — dedup threshold
    static constexpr int    kSwpS     = 180;      // samples for S sweep
    static constexpr int    kSwpA     = 180;      // samples for α sweep
};
