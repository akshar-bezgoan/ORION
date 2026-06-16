#include <iostream>
#include <iomanip>
#include <cmath>
#include <string>

#include "forward_kinematics.hpp"
#include "inverse_kinematics.hpp"
#include "robot_model.hpp"
#include "types.hpp"

static int g_pass = 0;
static int g_fail = 0;

static void record(const std::string& name, bool ok, const std::string& note = "")
{
    std::cout << (ok ? "[PASS]" : "[FAIL]") << " " << name;
    if (!note.empty()) {
        std::cout << " — " << note;
    }
    std::cout << "\n";
    ok ? ++g_pass : ++g_fail;
}

static bool near(const JointVector& a, const JointVector& b, double tol = 1e-6)
{
    if (a.size() != b.size()) {
        return false;
    }
    return (a - b).cwiseAbs().maxCoeff() < tol;
}

static bool verifyRoundTrip(RobotModel& model, const Position& target)
{
    InverseKinematics ik;
    ForwardKinematics fk;
    IKCandidates candidates = ik.solveAll(model, target);

    if (candidates.solutions.empty()) {
        return false;
    }

    for (const auto& solution : candidates.solutions) {
        if (solution.size() < 4) {
            continue;
        }

        JointVector q4(4);
        q4 << solution(0), solution(1), solution(2), solution(3);
        const PoseMatrix T = fk.solve(model, q4);
        const double err = std::sqrt(std::pow(T(0, 3) - std::sqrt(target(0) * target(0) + target(1) * target(1)), 2) +
                                     std::pow(T(1, 3) - target(2), 2));
        if (err < InverseKinematics::kPosTol * 10.0) {
            return true;
        }
    }

    return false;
}

static void test_allZero(RobotModel& model)
{
    std::cout << "\n=== IK Test: all joints zero ===\n";

    JointVector q_ref(4);
    q_ref.setZero();

    ForwardKinematics fk;
    PoseMatrix T = fk.solve(model, q_ref);

    Position target(3);
    target << T(0, 3), 0.0, T(1, 3);

    bool ok = verifyRoundTrip(model, target);
    record("all_zero round-trip", ok);
}

static void test_baseYaw(RobotModel& model)
{
    std::cout << "\n=== IK Test: base yaw extraction ===\n";

    JointVector q_ref(4);
    q_ref << 0.2, 0.1, 1.2, 0.3;

    ForwardKinematics fk;
    PoseMatrix T = fk.solve(model, q_ref);
    double px = T(0, 3);
    double py = T(1, 3);
    const double psi = 60.0 * M_PI / 180.0;

    Position target(3);
    target << px * std::cos(psi), px * std::sin(psi), py;

    InverseKinematics ik;
    IKCandidates candidates = ik.solveAll(model, target);

    bool foundPsi = false;
    for (const auto& solution : candidates.solutions) {
        if (solution.size() == 5 && std::abs(solution(4) - psi) < 1e-3) {
            foundPsi = true;
            break;
        }
    }

    record("base_yaw extracted", foundPsi, "expected psi = 60 deg");
}

static void test_reachableTargets(RobotModel& model)
{
    std::cout << "\n=== IK Test: reachable targets ===\n";

    std::vector<JointVector> references = {
        (JointVector(4) << 0.0, 0.0, 0.0, 0.0).finished(),
        (JointVector(4) << M_PI/4.0, 0.0, 0.0, 0.0).finished(),
        (JointVector(4) << -33.7 * M_PI / 180.0, -0.9 * M_PI / 180.0, 76.3 * M_PI / 180.0, 0.0).finished(),
        (JointVector(4) << 0.3, 0.2, 0.5, 0.8).finished(),
        (JointVector(4) << -0.70, -0.22, 2.60, -0.90).finished(),
    };

    ForwardKinematics fk;
    for (size_t i = 0; i < references.size(); ++i) {
        PoseMatrix T = fk.solve(model, references[i]);
        Position target(3);
        target << T(0, 3), 0.0, T(1, 3);

        bool ok = verifyRoundTrip(model, target);
        record("reachable target " + std::to_string(i + 1), ok);
    }
}

static void test_unreachableTarget(RobotModel& model)
{
    std::cout << "\n=== IK Test: unreachable target ===\n";

    Position target(3);
    target << 2.0, 0.0, 0.0;

    InverseKinematics ik;
    IKCandidates candidates = ik.solveAll(model, target);

    record("unreachable target yields no solutions", candidates.solutions.empty());
}

static void test_checkUtility(RobotModel& model)
{
    std::cout << "\n=== IK Test: check() utility ===\n";

    JointVector q_ref(4);
    q_ref << 0.5, 0.3, 1.0, 0.4;

    ForwardKinematics fk;
    PoseMatrix T = fk.solve(model, q_ref);

    Position target(3);
    target << T(0, 3), 0.0, T(1, 3);

    InverseKinematics ik;
    bool ok = ik.check(model, target, q_ref);
    record("check() accepts valid configuration", ok);

    JointVector q_bad = q_ref;
    q_bad(0) += 0.5;
    bool reject = !ik.check(model, target, q_bad);
    record("check() rejects invalid configuration", reject);
}

static void test_multipleSolutions(RobotModel& model)
{
    std::cout << "\n=== IK Test: multiple solutions ===\n";

    JointVector target_q(4);
    target_q << 0.0, 0.2, 1.0, 0.0;

    ForwardKinematics fk;
    PoseMatrix T = fk.solve(model, target_q);

    Position target(3);
    target << T(0, 3), 0.0, T(1, 3);

    InverseKinematics ik;
    IKCandidates candidates = ik.solveAll(model, target);

    bool enoughSolutions = candidates.solutions.size() >= 2;
    record("multiple solutions found", enoughSolutions);

    bool allValid = true;
    for (const auto& solution : candidates.solutions) {
        if (solution.size() < 4) {
            allValid = false;
            break;
        }
        JointVector q4(4);
        q4 << solution(0), solution(1), solution(2), solution(3);
        PoseMatrix result = fk.solve(model, q4);
        double err = std::sqrt(std::pow(result(0, 3) - T(0, 3), 2) + std::pow(result(1, 3) - T(1, 3), 2));
        if (err > InverseKinematics::kPosTol * 10.0) {
            allValid = false;
            break;
        }
    }
    record("all solutions validate with forward kinematics", allValid);
}

static void test_determinism(RobotModel& model)
{
    std::cout << "\n=== IK Test: determinism ===\n";

    Position target(3);
    target << 0.25, 0.0, 0.10;

    InverseKinematics ik;
    IKCandidates first = ik.solveAll(model, target);
    IKCandidates second = ik.solveAll(model, target);

    bool sameCount = first.solutions.size() == second.solutions.size();
    record("deterministic solution count", sameCount);
}

int main()
{
    RobotModel model;

    test_allZero(model);
    test_baseYaw(model);
    test_reachableTargets(model);
    test_unreachableTarget(model);
    test_checkUtility(model);
    test_multipleSolutions(model);
    test_determinism(model);

    std::cout << "\nRESULTS: " << g_pass << " passed, " << g_fail << " failed\n";
    return g_fail == 0 ? 0 : 1;
}
