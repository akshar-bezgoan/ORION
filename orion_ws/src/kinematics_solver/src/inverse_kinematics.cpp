#include "inverse_kinematics.hpp"
#include "forward_kinematics.hpp"

#include <cmath>
#include <algorithm>
#include <limits>

constexpr double kTwoPi = 2.0 * M_PI;

inline double wrapPi(double a)
{
    while (a >  M_PI) a -= kTwoPi;
    while (a <= -M_PI) a += kTwoPi;
    return a;
}

inline JointVector makeQ(double q0, double q1, double q2, double q3)
{
    JointVector q(4);
    q << q0, q1, q2, q3;
    return q;
}

inline bool nearlyEqual(const JointVector& a, const JointVector& b, double tol)
{
    return (a - b).cwiseAbs().maxCoeff() < tol;
}

struct JointPair { double q0, q1; };

int solve2R(double L1, double L2,
            double ex, double ey,
            JointPair out[2])
{
    const double d2    = ex * ex + ey * ey;
    const double cos_q1 = (d2 - L1 * L1 - L2 * L2) / (2.0 * L1 * L2);

    if (cos_q1 > 1.0 + 1e-9 || cos_q1 < -1.0 - 1e-9)
        return 0; 

    const double clamped = std::max(-1.0, std::min(1.0, cos_q1));
    const double sin_q1_pos = std::sqrt(std::max(0.0, 1.0 - clamped * clamped));

    int count = 0;
    for (int sign = +1; sign >= -1; sign -= 2) {
        const double s1 = sign * sin_q1_pos;
        const double q1 = std::atan2(s1, clamped);

        const double q0 = std::atan2(ey, ex)
                        - std::atan2(L2 * s1, L1 + L2 * clamped);

        out[count++] = {q0, q1};

        if (sin_q1_pos < 1e-9) break;
    }
    return count;
}

void tryCandidate(
        double L1, double L2, double L3, double L4,
        double px, double py,
        double S, double alpha,
        const RobotModel& model,
        std::vector<JointVector>& out)
{
    // Wrist centre (remove L4 along tool direction S)
    const double wx = px - L4 * std::cos(S);
    const double wy = py - L4 * std::sin(S);

    // Elbow centre (remove L3 along wrist direction α)
    const double ex = wx - L3 * std::cos(alpha);
    const double ey = wy - L3 * std::sin(alpha);

    // Solve 2R for (q0, q1)
    JointPair pairs[2];
    const int n = solve2R(L1, L2, ex, ey, pairs);

    for (int i = 0; i < n; ++i) {
        const double q0 = wrapPi(pairs[i].q0);
        const double q1 = wrapPi(pairs[i].q1);
        const double q2 = wrapPi(alpha - pairs[i].q0 - pairs[i].q1);
        const double q3 = wrapPi(S - alpha);

        JointVector q = makeQ(q0, q1, q2, q3);

        // Joint-limit check
        Eigen::Vector4d qv(q0, q1, q2, q3);
        if (!model.isWithinLimits(qv)) continue;

        //Skip if similar solutions exist
        bool dup = false;
        for (const auto& existing : out) {
            if (nearlyEqual(existing, q, InverseKinematics::kJointTol)) {
                dup = true;
                break;
            }
        }
        if (!dup) out.push_back(q);
    }
}


IKCandidates InverseKinematics::solveAll(const RobotModel& model, const Position& target) const
{
    if (target.size() != 3) return {};

    //target x,y,z
    const double tx = target(0);
    const double ty = target(1);
    const double tz = target(2);

    const double psi = std::atan2(ty, tx);   // required base yaw [rad]
    const double r   = std::sqrt(tx * tx + ty * ty);  // horizontal reach [m]

    const double px = r;
    const double py = tz;
    const double L1 = model.shoulder_h_;
    const double L2 = model.upper_arm_l_;
    const double L3 = model.forearm_l_;
    const double L4 = model.gripper_l_;

    const double dist = std::sqrt(px * px + py * py);
    const double rMax = L1 + L2 + L3 + L4;
    if (dist > rMax + kPosTol) return {};

    const double S_min = -M_PI;
    const double S_max =  M_PI;
    const double a_min = -0.98;  
    const double a_max =  4.57;   

    const double dS = (S_max - S_min) / static_cast<double>(kSwpS);
    const double dA = (a_max - a_min) / static_cast<double>(kSwpA);

    const double dir = std::atan2(py, px);  

    std::vector<double> S_vals;
    S_vals.reserve(kSwpS + 8);
    for (int i = 0; i <= kSwpS; ++i)
        S_vals.push_back(S_min + i * dS);
    // Sentinels: target bearing, cardinal angles
    for (double extra : {dir, 0.0, M_PI_2, -M_PI_2, M_PI, -M_PI})
        S_vals.push_back(extra);

    std::vector<double> a_vals;
    a_vals.reserve(kSwpA + 8);
    for (int i = 0; i <= kSwpA; ++i)
        a_vals.push_back(a_min + i * dA);
    // Sentinels
    for (double extra : {dir, 0.0, M_PI_2, -M_PI_2})
        a_vals.push_back(extra);

    std::vector<JointVector> raw;
    raw.reserve(1024);

    for (const double S : S_vals) {
        for (const double alpha : a_vals) {
            tryCandidate(L1, L2, L3, L4, px, py, S, alpha, model, raw);
        }
    }

    ForwardKinematics fk;
    IKCandidates result;

    for (const auto& q : raw) {
        const PoseMatrix T = fk.solve(model, q);
        const double err_x = T(0, 3) - px;
        const double err_y = T(1, 3) - py;
        const double err   = std::sqrt(err_x * err_x + err_y * err_y);

        if (err > kPosTol) continue;

        JointVector full(5);
        full << q(0), q(1), q(2), q(3), psi;
        result.solutions.push_back(full);
    }

    return result;
}

JointVector InverseKinematics::solve(RobotModel& model, Position end_position) const
{
    const IKCandidates candidates = solveAll(model, end_position);
    if (candidates.solutions.empty()) {
        JointVector zero(5);
        zero.setZero();
        return zero;
    }
    return candidates.solutions.front();
}

bool InverseKinematics::check(RobotModel& model, Position end_position, JointVector& q) const
{
    if (end_position.size() != 3 || q.size() < 4) return false;

    const double tx = end_position(0);
    const double ty = end_position(1);
    const double tz = end_position(2);
    const double px = std::sqrt(tx * tx + ty * ty);
    const double py = tz;

    ForwardKinematics fk;
    // Use only the first 4 elements if q has 5 (base yaw appended)
    JointVector q4(4);
    q4 << q(0), q(1), q(2), q(3);
    const PoseMatrix T = fk.solve(model, q4);

    const double err = std::sqrt(std::pow(T(0, 3) - px, 2) +
                                 std::pow(T(1, 3) - py, 2));
    return err < kPosTol;
}
