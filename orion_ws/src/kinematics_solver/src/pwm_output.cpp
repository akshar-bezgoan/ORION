#include "pwm_output.hpp"

#include <cmath>
#include <algorithm>

MinimumJerkCoefficients computeMinimumJerkCoefficients(const JointVector& start,
                                                       const JointVector& goal,
                                                       double duration)
{
    const int dof = static_cast<int>(start.size());
    MinimumJerkCoefficients coeffs;
    coeffs.a0 = start;
    coeffs.a1 = JointVector::Zero(dof);
    coeffs.a2 = JointVector::Zero(dof);
    coeffs.a3 = JointVector::Zero(dof);
    coeffs.a4 = JointVector::Zero(dof);
    coeffs.a5 = JointVector::Zero(dof);

    const double T = std::max(duration, 1e-6);
    const double T2 = T * T;
    const double T3 = T2 * T;
    const double T4 = T3 * T;
    const double T5 = T4 * T;

    for (int i = 0; i < dof; ++i) {
        const double dq = goal[i] - start[i];
        coeffs.a3[i] = 10.0 * dq / T3;
        coeffs.a4[i] = -15.0 * dq / T4;
        coeffs.a5[i] = 6.0 * dq / T5;
    }

    return coeffs;
}

JointVector evaluateMinimumJerkPosition(const MinimumJerkCoefficients& coeffs,
                                        double time)
{
    const int dof = static_cast<int>(coeffs.a0.size());
    JointVector position = coeffs.a0;
    const double t = time;
    const double t2 = t * t;
    const double t3 = t2 * t;
    const double t4 = t3 * t;
    const double t5 = t4 * t;

    position += coeffs.a1 * t;
    position += coeffs.a2 * t2;
    position += coeffs.a3 * t3;
    position += coeffs.a4 * t4;
    position += coeffs.a5 * t5;

    return position;
}

JointVector evaluateMinimumJerkAcceleration(const MinimumJerkCoefficients& coeffs,
                                            double time)
{
    const int dof = static_cast<int>(coeffs.a0.size());
    JointVector acceleration = JointVector::Zero(dof);
    const double t = time;
    const double t2 = t * t;

    acceleration += coeffs.a2 * 2.0;
    acceleration += coeffs.a3 * (6.0 * t);
    acceleration += coeffs.a4 * (12.0 * t2);
    acceleration += coeffs.a5 * (20.0 * t2 * t);

    return acceleration;
}

std::vector<JointVector> generateMinimumJerkAccelerationProfile(const JointVector& start,
                                                                 const JointVector& goal,
                                                                 double duration,
                                                                 int samples)
{
    const int sampleCount = std::max(2, samples);
    std::vector<JointVector> profile;
    profile.reserve(sampleCount);

    MinimumJerkCoefficients coeffs = computeMinimumJerkCoefficients(start, goal, duration);
    const double dt = duration / static_cast<double>(sampleCount - 1);

    for (int sample = 0; sample < sampleCount; ++sample) {
        const double time = sample * dt;
        profile.push_back(evaluateMinimumJerkAcceleration(coeffs, time));
    }

    return profile;
}

JointVector computeTorque(const RobotModel& model,
                          const JointVector& acceleration)
{
    const int dof = static_cast<int>(acceleration.size());
    JointVector torque = JointVector::Zero(dof);
    const int modelDOF = model.getDOF();

    for (int joint = 0; joint < dof; ++joint) {
        if (joint >= modelDOF) {
            torque[joint] = 0.0;
            continue;
        }

        double torqueProxy = 0.0;
        for (int link = joint; link < modelDOF; ++link) {
            const double mass = model.getLinkMass(link);
            const double length = model.getLinkLength(link);
            torqueProxy += std::abs(acceleration[joint]) * mass * length;
        }
        torque[joint] = torqueProxy;
    }

    return torque;
}

std::vector<JointVector> generateTorqueProfile(const RobotModel& model,
                                               const JointVector& start,
                                               const JointVector& goal,
                                               double duration,
                                               int samples)
{
    const std::vector<JointVector> accelerations = generateMinimumJerkAccelerationProfile(start, goal, duration, samples);
    std::vector<JointVector> torqueProfile;
    torqueProfile.reserve(accelerations.size());

    for (const JointVector& acceleration : accelerations) {
        torqueProfile.push_back(computeTorque(model, acceleration));
    }

    return torqueProfile;
}
