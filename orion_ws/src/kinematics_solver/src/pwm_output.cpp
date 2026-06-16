#include "pwm_output.hpp"
#include "types.hpp"

#include <cmath>
#include <algorithm>

MinimumJerkCoefficients getMJCoeff(const JointVector& start,
                                   const JointVector& goal,
                                   double duration)
{
    const int dof = static_cast<int>(start.size());
    MinimumJerkCoefficients coef;
    coef.a0 = start;
    coef.a1 = JointVector::Zero(dof);
    coef.a2 = JointVector::Zero(dof);
    coef.a3 = JointVector::Zero(dof);
    coef.a4 = JointVector::Zero(dof);
    coef.a5 = JointVector::Zero(dof);

    const double T = std::max(duration, 1e-6);  // Duration
    const double T2 = T * T;
    const double T3 = T2 * T;
    const double T4 = T3 * T;
    const double T5 = T4 * T;

    for (int i = 0; i < dof; ++i) {
        const double dq = goal[i] - start[i];
        coef.a3[i] = 10.0 * dq / T3;
        coef.a4[i] = -15.0 * dq / T4;
        coef.a5[i] = 6.0 * dq / T5;
    }

    return coef;
}

JointVector evalMJPos(const MinimumJerkCoefficients& coef,
                      double t)
{
    const int dof = static_cast<int>(coef.a0.size());
    JointVector pos = coef.a0;
    const double t2 = t * t;
    const double t3 = t2 * t;
    const double t4 = t3 * t;
    const double t5 = t4 * t;

    pos += coef.a1 * t;
    pos += coef.a2 * t2;
    pos += coef.a3 * t3;
    pos += coef.a4 * t4;
    pos += coef.a5 * t5;

    return pos;
}

JointVector evalMJAccel(const MinimumJerkCoefficients& coef,
                        double t)
{
    const int dof = static_cast<int>(coef.a0.size());
    JointVector accel = JointVector::Zero(dof);
    const double t2 = t * t;

    accel += coef.a2 * 2.0;
    accel += coef.a3 * (6.0 * t);
    accel += coef.a4 * (12.0 * t2);
    accel += coef.a5 * (20.0 * t2 * t);

    return accel;
}

// Generate minimum jerk acceleration profile
std::vector<JointVector> genMJProfile(const JointVector& start,
                                       const JointVector& goal,
                                       double duration,
                                       int samples)
{
    const int n = std::max(2, samples);  // num samples
    std::vector<JointVector> prof;
    prof.reserve(n);

    MinimumJerkCoefficients coef = getMJCoeff(start, goal, duration);
    const double dt = duration / static_cast<double>(n - 1);

    for (int i = 0; i < n; ++i) {
        prof.push_back(evalMJAccel(coef, i * dt));
    }

    return prof;
}

JointVector calcTau(const RobotModel& model,
                    const JointVector& accel)
{
    const int dof = static_cast<int>(accel.size());
    JointVector tau = JointVector::Zero(dof);
    const int n_dof = model.getDOF();

    for (int j = 0; j < dof; ++j) {
        if (j >= n_dof) {
            tau[j] = 0.0;
            continue;
        }

        double sum = 0.0;  // Torque accumulator
        for (int k = j; k < n_dof; ++k) {
            const double m = model.getLinkMass(k);
            const double l = model.getLinkLength(k);
            sum += std::abs(accel[j]) * m * l;
        }
        tau[j] = sum;
    }

    return tau;
}

std::vector<JointVector> genTauProfile(const RobotModel& model,
                                        const JointVector& start,
                                        const JointVector& goal,
                                        double duration,
                                        int samples)
{
    const std::vector<JointVector> accels = genMJProfile(start, goal, duration, samples);
    std::vector<JointVector> tau_prof;
    tau_prof.reserve(accels.size());

    for (const JointVector& a : accels) {
        tau_prof.push_back(calcTau(model, a));
    }

    return tau_prof;
}
