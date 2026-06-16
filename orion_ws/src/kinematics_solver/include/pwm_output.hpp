#pragma once

#include "robot_model.hpp"
#include "types.hpp"

#include <vector>

MinimumJerkCoefficients getMJCoeff(const JointVector& start,
                                   const JointVector& goal,
                                   double duration);

JointVector evalMJPos(const MinimumJerkCoefficients& coef,
                      double t);

JointVector evalMJAccel(const MinimumJerkCoefficients& coef,
                        double t);

std::vector<JointVector> genMJProfile(const JointVector& start,
                                       const JointVector& goal,
                                       double duration,
                                       int samples);

JointVector calcTau(const RobotModel& model,
                    const JointVector& accel);

std::vector<JointVector> genTauProfile(const RobotModel& model,
                                        const JointVector& start,
                                        const JointVector& goal,
                                        double duration,
                                        int samples);
