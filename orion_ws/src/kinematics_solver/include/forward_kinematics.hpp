#pragma once
#include "types.hpp"
#include "robot_model.hpp"

class ForwardKinematics{
    public:
        PoseMatrix solve(
            const RobotModel& model,
            const JointVector& q) const;
};
