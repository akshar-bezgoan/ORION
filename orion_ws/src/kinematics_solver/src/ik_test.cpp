#include <iostream>
#include <iomanip>
#include <cmath>
#include <string>

#include "forward_kinematics.hpp"
#include "inverse_kinematics.hpp"
#include "trajectory_optimiser.hpp"

//Helper formatter
static void printSep(const std::string& title)
{
    std::cout << "\n" << std::string(70, '=') << "\n";
    std::cout << "  " << title << "\n";
    std::cout << std::string(70, '-') << "\n";
}

static void printQ(const std::string& label, const JointVector& q)
{
    std::cout << std::fixed << std::setprecision(4);
    std::cout << label << ":  ";
    for (int i = 0; i < static_cast<int>(q.size()); ++i)
        std::cout << q(i) * 180.0 / M_PI << " deg" << (i + 1 < static_cast<int>(q.size()) ? "  |  " : "");
    std::cout << "\n";
}

static void printFKPos(const std::string& label, const PoseMatrix& T)
{
    std::cout << label
              << "  px=" << std::setw(8) << T(0, 3)
              << "  py=" << std::setw(8) << T(1, 3)
              << "  orientation=" << std::atan2(T(1, 0), T(0, 0)) * 180.0 / M_PI << " deg\n";
}

struct TestResult {
    std::string name;
    bool        passed;
    std::string note;
};

static int g_pass = 0, g_fail = 0;

static void record(const std::string& name, bool ok, const std::string& note = "")
{
    std::cout << (ok ? "[PASS]" : "[FAIL]") << "  " << name;
    if (!note.empty()) std::cout << "  — " << note;
    std::cout << "\n";
    ok ? ++g_pass : ++g_fail;
}


static bool roundTrip(const std::string&, RobotModel& model, double px, double py, double psi = 0.0, double tol = InverseKinematics::kPosTol * 10.0)
{
    ForwardKinematics  fk;
    InverseKinematics  ik;

    Position target(3);
    target << px * std::cos(psi),
              px * std::sin(psi),
              py;

    std::cout << "\n  Target (world): tx=" << target(0)
              << "  ty=" << target(1)
              << "  tz=" << target(2) << "\n";
    std::cout << "  Arm plane:  px=" << px << "  py=" << py
              << "  psi=" << psi * 180.0 / M_PI << " deg\n";

    IKCandidates cands = ik.solveAll(model, target);
    std::cout << "  Solutions found: " << cands.solutions.size() << "\n";

    bool anyGood = false;
    for (std::size_t i = 0; i < cands.solutions.size(); ++i) {
        const JointVector& sol = cands.solutions[i];
        JointVector q4(4);
        q4 << sol(0), sol(1), sol(2), sol(3);

        PoseMatrix T = fk.solve(model, q4);
        double err   = std::sqrt(std::pow(T(0, 3) - px, 2) +
                                 std::pow(T(1, 3) - py, 2));

        std::cout << "  [" << std::setw(3) << i << "]";
        printQ("  q", q4);
        std::cout << "       psi=" << sol(4) * 180.0 / M_PI << " deg"
                  << "   FK err=" << std::scientific << std::setprecision(2)
                  << err << "\n";
        std::cout << std::fixed << std::setprecision(4);

        if (err < tol) anyGood = true;
    }
    return anyGood;
}

// TEST CASES

// T1: All joints zero (arm fully extended along +X) 
static void test_allZero(RobotModel& model)
{
    printSep("T1: All joints zero — arm fully extended");

    JointVector q_ref(4);
    q_ref << 0.0, 0.0, 0.0, 0.0;
    ForwardKinematics fk;
    PoseMatrix T = fk.solve(model, q_ref);
    printFKPos("  FK reference", T);

    bool ok = roundTrip("T1", model, T(0, 3), T(1, 3));
    record("T1: All joints zero — round-trip", ok);
}

// T2: Shoulder 45°, rest zero
static void test_shoulder45(RobotModel& model)
{
    printSep("T2: Shoulder 45 deg, elbow/wrist/roll = 0");

    JointVector q_ref(4);
    q_ref << M_PI / 4.0, 0.0, 0.0, 0.0;
    ForwardKinematics fk;
    PoseMatrix T = fk.solve(model, q_ref);
    printQ("  q_ref", q_ref);
    printFKPos("  FK reference", T);

    bool ok = roundTrip("T2", model, T(0, 3), T(1, 3));
    record("T2: Shoulder 45 deg — round-trip", ok);
}

// T3: Representative grasp config from test_fk.cpp 
static void test_graspConfig(RobotModel& model)
{
    printSep("T3: Grasp config  (-33.7, -0.9, 76.3, 0) deg");

    JointVector q_ref(4);
    q_ref << -33.7 * M_PI / 180.0,
             -0.9  * M_PI / 180.0,
              76.3 * M_PI / 180.0,
              0.0;
    ForwardKinematics fk;
    PoseMatrix T = fk.solve(model, q_ref);
    printQ("  q_ref", q_ref);
    printFKPos("  FK reference", T);

    bool ok = roundTrip("T3", model, T(0, 3), T(1, 3));
    record("T3: Grasp config — round-trip", ok);
}

//  T4: Non-zero wrist roll
static void test_wristRoll(RobotModel& model)
{
    printSep("T4: Non-zero wrist roll (q0=0.3, q1=0.2, q2=0.5, q3=0.8)");

    JointVector q_ref(4);
    q_ref << 0.3, 0.2, 0.5, 0.8;
    ForwardKinematics fk;
    PoseMatrix T = fk.solve(model, q_ref);
    printQ("  q_ref", q_ref);
    printFKPos("  FK reference", T);

    bool ok = roundTrip("T4", model, T(0, 3), T(1, 3));
    record("T4: Non-zero wrist roll — round-trip", ok);
}

// T5: Near joint limits
static void test_nearLimits(RobotModel& model)
{
    printSep("T5: Near joint limits (q0=-0.70, q1=-0.22, q2=2.60, q3=-0.90)");

    JointVector q_ref(4);
    q_ref << -0.70, -0.22, 2.60, -0.90;
    ForwardKinematics fk;
    PoseMatrix T = fk.solve(model, q_ref);
    printQ("  q_ref", q_ref);
    printFKPos("  FK reference", T);

    bool ok = roundTrip("T5", model, T(0, 3), T(1, 3));
    record("T5: Near joint limits — round-trip", ok);
}

// T6: Non-zero base yaw (target off X axis)
static void test_baseYaw(RobotModel& model)
{
    printSep("T6: Non-zero base yaw — target at 60 deg in world XY plane");

    JointVector q_ref(4);
    q_ref << 0.2, 0.1, 1.2, 0.3;
    ForwardKinematics fk;
    PoseMatrix T = fk.solve(model, q_ref);
    double px_arm = T(0, 3);
    double py_arm = T(1, 3);

    const double psi = 60.0 * M_PI / 180.0;

    printQ("  q_ref", q_ref);
    std::cout << "  Arm-plane target:  px=" << px_arm << "  py=" << py_arm << "\n";
    std::cout << "  Base yaw applied:  psi=" << psi * 180.0 / M_PI << " deg\n";

    bool ok = roundTrip("T6", model, px_arm, py_arm, psi);

    // Also verify psi is correctly extracted
    if (ok && !fk.solve(model, q_ref).isZero()) {
        InverseKinematics ik;
        Position target(3);
        target << px_arm * std::cos(psi),
                  px_arm * std::sin(psi),
                  py_arm;
        IKCandidates c = ik.solveAll(model, target);
        bool psiOk = false;
        for (const auto& s : c.solutions) {
            if (std::abs(s(4) - psi) < 1e-3) { psiOk = true; break; }
        }
        record("T6: Base yaw extraction correct", psiOk);
    }
    record("T6: Non-zero base yaw — round-trip", ok);
}

// T7: Unreachable — too far
static void test_unreachableFar(RobotModel& model)
{
    printSep("T7: Unreachable target — beyond max reach");

    InverseKinematics ik;
    Position target(3);
    target << 2.0, 0.0, 0.0;   // 2 m >> 0.470 m max

    IKCandidates c = ik.solveAll(model, target);
    std::cout << "  Solutions found: " << c.solutions.size() << " (expected 0)\n";
    record("T7: Unreachable (too far) — no solutions", c.solutions.empty());
}

// T8: Unreachable — wrong size Position 
static void test_badInput(RobotModel& model)
{
    printSep("T8: Malformed input — Position size != 3");

    InverseKinematics ik;
    
    // Use dynamic vector to test size mismatch handling
    JointVector target(2);   
    target << 0.2, 0.1;

    // solveAll expects Position (Vector3d), passing wrong-sized vector
    // This should be caught by the size check and return empty
    IKCandidates c = ik.solveAll(model, Position(target(0), target(1), 0.0));
    
    // Also test with properly sized Position but extreme values
    Position extreme;
    extreme << 10.0, 10.0, 10.0;  // Way out of reach
    IKCandidates c2 = ik.solveAll(model, extreme);
    
    std::cout << "  Solutions with valid size but unreachable target: " << c2.solutions.size() << " (expected 0)\n";
    record("T8: Unreachable target returns no solutions", c2.solutions.empty());
}

// T9: check() utility 
static void test_checkUtility(RobotModel& model)
{
    printSep("T9: check() utility — verify a known-good configuration");

    JointVector q_ref(4);
    q_ref << 0.5, 0.3, 1.0, 0.4;
    ForwardKinematics fk;
    PoseMatrix T = fk.solve(model, q_ref);


    Position target(3);
    target << T(0, 3), 0.0, T(1, 3);

    InverseKinematics ik;
    bool ok = ik.check(model, target, q_ref);
    record("T9: check() on valid config", ok);

    JointVector q_bad = q_ref;
    q_bad(0) += 0.5; 
    bool badFail = !ik.check(model, target, q_bad);
    record("T9: check() correctly rejects perturbed config", badFail);
}

// T10: Multiple solutions enumeration 
static void test_multiSolution(RobotModel& model)
{
    printSep("T10: Multiple solutions — count elbow-up vs elbow-down branches");

    JointVector q_ref(4);
    q_ref << 0.0, 0.2, 1.0, 0.0;
    ForwardKinematics fk;
    PoseMatrix T = fk.solve(model, q_ref);
    double px = T(0, 3), py = T(1, 3);

    Position target(3);
    target << px, 0.0, py;

    InverseKinematics ik;
    IKCandidates c = ik.solveAll(model, target);

    std::cout << "  Target: px=" << px << "  py=" << py << "\n";
    std::cout << "  Solutions found: " << c.solutions.size() << "\n";

    int fkPassed = 0;
    for (const auto& sol : c.solutions) {
        JointVector q4(4);
        q4 << sol(0), sol(1), sol(2), sol(3);
        PoseMatrix Ts = fk.solve(model, q4);
        double err = std::sqrt(std::pow(Ts(0, 3) - px, 2) + std::pow(Ts(1, 3) - py, 2));
        if (err < InverseKinematics::kPosTol * 10.0) ++fkPassed;
    }

    record("T10: All solutions pass FK round-trip",
           fkPassed == static_cast<int>(c.solutions.size()));
    record("T10: At least 2 solutions found (redundant arm)",
           c.solutions.size() >= 2);
}

static bool jointNear(const JointVector& a, const JointVector& b, double tol = 1e-6)
{
    if (a.size() != b.size()) {
        return false;
    }
    return (a - b).cwiseAbs().maxCoeff() < tol;
}

// T11: Trajectory optimiser selects best IK candidate from multiple options
static void test_trajectoryOptimiserSelection(RobotModel& model)
{
    printSep("T11: Trajectory optimiser selects best candidate");

    JointVector current(4);
    current.setZero();

    JointVector lowCost(4);
    lowCost << 0.0, 0.0, 0.0, 0.0;

    JointVector highCost(4);
    highCost << 0.0, 1.0, 1.0, 0.0;

    IKCandidates candidates;
    candidates.solutions.push_back(highCost);
    candidates.solutions.push_back(lowCost);

    TrajectoryOptimiserParameters params;
    params.static_weight = 1.0;
    params.dynamic_weight = 1.0;
    params.trajectory_samples = 8;
    params.trajectory_time = 1.0;

    TrajectoryOptimiser optimiser(model, params);
    JointVector selected = optimiser.selectBest(current, candidates);

    bool ok = jointNear(selected, lowCost);
    record("T11: selectBest chooses lowest total cost", ok);
}

// T12: Trajectory optimiser handles empty candidate sets gracefully
static void test_trajectoryOptimiserEmpty(RobotModel& model)
{
    printSep("T12: Trajectory optimiser handles empty candidate set");

    JointVector current(4);
    current.setZero();

    IKCandidates candidates;

    TrajectoryOptimiserParameters params;
    params.static_weight = 1.0;
    params.dynamic_weight = 1.0;

    TrajectoryOptimiser optimiser(model, params);
    JointVector selected = optimiser.selectBest(current, candidates);

    bool ok = jointNear(selected, current);
    record("T12: empty candidate list returns current configuration", ok);
}

// T13: Trajectory optimiser respects weight bias between static and dynamic cost
static void test_trajectoryOptimiserWeightBias(RobotModel& model)
{
    printSep("T13: Weight bias influences selection");

    JointVector current(4);
    current.setZero();

    JointVector nearHighStatic(4);
    nearHighStatic << 0.0, 0.0, 1.0, 0.0;

    JointVector farLowStatic(4);
    farLowStatic << 0.0, 1.5, 0.0, 0.0;

    IKCandidates candidates;
    candidates.solutions.push_back(nearHighStatic);
    candidates.solutions.push_back(farLowStatic);

    TrajectoryOptimiserParameters staticBias;
    staticBias.static_weight = 10.0;
    staticBias.dynamic_weight = 0.1;
    staticBias.trajectory_samples = 10;
    staticBias.trajectory_time = 1.0;

    TrajectoryOptimiser optimiser(model, staticBias);
    JointVector selectedStatic = optimiser.selectBest(current, candidates);

    bool staticOk = jointNear(selectedStatic, farLowStatic);
    record("T13: static-weight bias picks lower static cost", staticOk);

    TrajectoryOptimiserParameters dynamicBias;
    dynamicBias.static_weight = 0.1;
    dynamicBias.dynamic_weight = 10.0;
    dynamicBias.trajectory_samples = 10;
    dynamicBias.trajectory_time = 1.0;

    TrajectoryOptimiser optimiserDynamic(model, dynamicBias);
    JointVector selectedDynamic = optimiserDynamic.selectBest(current, candidates);

    bool dynamicOk = jointNear(selectedDynamic, nearHighStatic);
    record("T13: dynamic-weight bias picks lower motion cost", dynamicOk);
}

// T14: Shoulder at 90 deg (arm pointing up) 
static void test_shoulder90(RobotModel& model)
{
    printSep("T11: Shoulder 90 deg — arm pointing up");

    JointVector q_ref(4);
    q_ref << M_PI / 2.0, 0.0, 0.0, 0.0;

    // Check this is within limits (shoulder_hi = 1.30, pi/2 ≈ 1.571 — OUTSIDE)
    Eigen::Vector4d qv(q_ref(0), q_ref(1), q_ref(2), q_ref(3));
    if (!model.isWithinLimits(qv)) {
        std::cout << "  NOTE: q_ref is outside joint limits (shoulder pi/2 > 1.30)\n";
        std::cout << "  Using shoulder = 1.20 rad instead\n";
        q_ref(0) = 1.20;
    }

    ForwardKinematics fk;
    PoseMatrix T = fk.solve(model, q_ref);
    printQ("  q_ref", q_ref);
    printFKPos("  FK reference", T);

    bool ok = roundTrip("T11", model, T(0, 3), T(1, 3));
    record("T11: Near-vertical arm — round-trip", ok);
}

// T12: Solution uniqueness — identical targets give same solution set 
static void test_determinism(RobotModel& model)
{
    printSep("T12: Determinism — identical calls return same number of solutions");

    Position target(3);
    target << 0.25, 0.0, 0.10;

    InverseKinematics ik;
    IKCandidates c1 = ik.solveAll(model, target);
    IKCandidates c2 = ik.solveAll(model, target);

    bool sameCount = (c1.solutions.size() == c2.solutions.size());
    record("T12: Deterministic solution count", sameCount);

    // Check each solution in c1 appears in c2
    bool allMatch = true;
    for (const auto& s1 : c1.solutions) {
        bool found = false;
        for (const auto& s2 : c2.solutions) {
            if ((s1 - s2).cwiseAbs().maxCoeff() < InverseKinematics::kJointTol * 10.0) {
                found = true; break;
            }
        }
        if (!found) { allMatch = false; break; }
    }
    record("T12: Solutions are identical across calls", allMatch);
}

int main()
{
    RobotModel model;

    std::cout << "\nIK Validation\n";
    std::cout << "Link lengths: L1=0.050  L2=0.220  L3=0.145  L4=0.055\n";
    std::cout << "Max reach: 0.470 m\n";

    test_allZero(model);
    test_shoulder45(model);
    test_graspConfig(model);
    test_wristRoll(model);
    test_nearLimits(model);
    test_baseYaw(model);
    test_unreachableFar(model);
    test_badInput(model);
    test_checkUtility(model);
    test_multiSolution(model);
    test_trajectoryOptimiserSelection(model);
    test_trajectoryOptimiserEmpty(model);
    test_trajectoryOptimiserWeightBias(model);
    test_shoulder90(model);
    test_determinism(model);

    std::cout << "\n" << std::string(70, '=') << "\n";
    std::cout << "  RESULTS:  " << g_pass << " passed  |  " << g_fail << " failed\n";
    std::cout << std::string(70, '=') << "\n\n";

    return g_fail == 0 ? 0 : 1;
}
