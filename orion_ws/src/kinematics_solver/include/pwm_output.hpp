#pragma once

#include "robot_model.hpp"
#include "types.hpp"

#include <vector>

struct MinimumJerkCoefficients {
    JointVector a0;
    JointVector a1;
    JointVector a2;
    JointVector a3;
    JointVector a4;
    JointVector a5;
};

MinimumJerkCoefficients computeMinimumJerkCoefficients(const JointVector& start,
                                                       const JointVector& goal,
                                                       double duration);

JointVector evaluateMinimumJerkPosition(const MinimumJerkCoefficients& coeffs,
                                        double time);

JointVector evaluateMinimumJerkAcceleration(const MinimumJerkCoefficients& coeffs,
                                            double time);

std::vector<JointVector> generateMinimumJerkAccelerationProfile(const JointVector& start,
                                                                 const JointVector& goal,
                                                                 double duration,
                                                                 int samples);

JointVector computeTorque(const RobotModel& model,
                          const JointVector& acceleration);

std::vector<JointVector> generateTorqueProfile(const RobotModel& model,
                                               const JointVector& start,
                                               const JointVector& goal,
                                               double duration,
                                               int samples);
