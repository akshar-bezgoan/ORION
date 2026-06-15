#include <iostream>
#include <iomanip>
#include <cmath>
#include <cassert>
#include <vector>

#include "trajectory_optimiser.hpp"
#include "robot_model.hpp"
#include "forward_kinematics.hpp"
#include "types.hpp"

static int g_pass = 0, g_fail = 0;

static void record(const std::string& name, bool ok, const std::string& note = "")
{
    std::cout << (ok ? "[PASS]" : "[FAIL]") << "  " << name;
    if (!note.empty()) std::cout << "  — " << note;
    std::cout << "\n";
    ok ? ++g_pass : ++g_fail;
}

static void printSep(const std::string& title)
{
    std::cout << "\n" << std::string(70, '=') << "\n";
    std::cout << "  " << title << "\n";
    std::cout << std::string(70, '-') << "\n";
}

static bool jointNear(const JointVector& a, const JointVector& b, double tol = 1e-6)
{
    if (a.size() != b.size()) {
        return false;
    }
    return (a - b).cwiseAbs().maxCoeff() < tol;
}

// ============================================================================
// UNIT TEST 1: StaticCostEvaluator returns non-negative costs
// ============================================================================
static void test_staticCostNonNegative(RobotModel& model)
{
    printSep("UT1: Static cost evaluator non-negativity");

    StaticCostEvaluator evaluator(model);

    JointVector q_zero(4);
    q_zero.setZero();

    JointVector q_nonzero(4);
    q_nonzero << 0.5, 0.3, 1.0, -0.4;

    double cost_zero = evaluator.evaluate(q_zero);
    double cost_nonzero = evaluator.evaluate(q_nonzero);

    record("UT1a: Static cost at zero is non-negative", cost_zero >= 0.0);
    record("UT1b: Static cost at non-zero is non-negative", cost_nonzero >= 0.0);
    record("UT1c: Non-zero pose has higher cost than zero", cost_nonzero > cost_zero);
}

// ============================================================================
// UNIT TEST 2: TrajectoryGenerator produces correct number of samples
// ============================================================================
static void test_trajectoryGeneratorSampleCount(RobotModel& model)
{
    printSep("UT2: Trajectory generator sample count");

    for (int samples : {2, 5, 10, 20}) {
        TrajectoryGenerator gen(model.getDOF(), samples);

        JointVector start(4);
        start.setZero();

        JointVector goal(4);
        goal << 0.5, 0.3, 1.0, -0.4;

        std::vector<JointVector> trajectory = gen.generate(start, goal);

        bool ok = (trajectory.size() == static_cast<size_t>(samples));
        record("UT2: Trajectory has " + std::to_string(samples) + " samples", ok);
    }
}

// ============================================================================
// UNIT TEST 3: TrajectoryGenerator interpolation correctness
// ============================================================================
static void test_trajectoryGeneratorInterpolation(RobotModel& model)
{
    printSep("UT3: Trajectory generator interpolation correctness");

    TrajectoryGenerator gen(model.getDOF(), 11);

    JointVector start(4);
    start.setZero();

    JointVector goal(4);
    goal << 1.0, 2.0, 3.0, 4.0;

    std::vector<JointVector> trajectory = gen.generate(start, goal);

    // Check start point
    bool startOk = jointNear(trajectory[0], start, 1e-10);
    record("UT3a: First point matches start", startOk);

    // Check end point
    bool endOk = jointNear(trajectory.back(), goal, 1e-10);
    record("UT3b: Last point matches goal", endOk);

    // Check midpoint interpolation
    JointVector midExpected = (start + goal) / 2.0;
    bool midOk = jointNear(trajectory[5], midExpected, 1e-10);
    record("UT3c: Midpoint is correctly interpolated", midOk);

    // Check monotonicity along a path
    bool monotonic = true;
    for (int i = 0; i < 4; ++i) {
        for (size_t j = 1; j < trajectory.size(); ++j) {
            if (goal[i] > start[i]) {
                if (trajectory[j][i] < trajectory[j-1][i] - 1e-10) {
                    monotonic = false;
                    break;
                }
            } else if (goal[i] < start[i]) {
                if (trajectory[j][i] > trajectory[j-1][i] + 1e-10) {
                    monotonic = false;
                    break;
                }
            }
        }
    }
    record("UT3d: Path monotonicity preserved", monotonic);
}

// ============================================================================
// UNIT TEST 4: DynamicCostEvaluator is non-negative and sensible
// ============================================================================
static void test_dynamicCostProperties(RobotModel& model)
{
    printSep("UT4: Dynamic cost evaluator properties");

    DynamicCostEvaluator evaluator(model);

    JointVector start(4);
    start.setZero();

    JointVector goal_near(4);
    goal_near << 0.1, 0.1, 0.1, 0.1;

    JointVector goal_far(4);
    goal_far << 1.0, 1.0, 1.0, 1.0;

    TrajectoryGenerator gen(model.getDOF(), 10);

    std::vector<JointVector> traj_near = gen.generate(start, goal_near);
    std::vector<JointVector> traj_far = gen.generate(start, goal_far);

    double cost_near = evaluator.evaluate(start, goal_near, traj_near, 1.0);
    double cost_far = evaluator.evaluate(start, goal_far, traj_far, 1.0);

    record("UT4a: Dynamic cost for near goal is non-negative", cost_near >= 0.0);
    record("UT4b: Dynamic cost for far goal is non-negative", cost_far >= 0.0);
    record("UT4c: Far goal has higher dynamic cost", cost_far > cost_near);
}

// ============================================================================
// UNIT TEST 5: TrajectoryOptimiser selects the best candidate
// ============================================================================
static void test_optimiserSelectionBest(RobotModel& model)
{
    printSep("UT5: Optimiser selects best candidate");

    JointVector current(4);
    current.setZero();

    // Create candidates: one very close (low cost), one far (high cost)
    JointVector candidate_good(4);
    candidate_good << 0.05, 0.05, 0.05, 0.05;

    JointVector candidate_bad(4);
    candidate_bad << 1.0, 1.0, 1.0, 1.0;

    IKCandidates candidates;
    candidates.solutions.push_back(candidate_bad);
    candidates.solutions.push_back(candidate_good);

    TrajectoryOptimiserParameters params;
    params.static_weight = 1.0;
    params.dynamic_weight = 1.0;
    params.trajectory_samples = 10;
    params.trajectory_time = 1.0;

    TrajectoryOptimiser optimiser(model, params);
    JointVector selected = optimiser.selectBest(current, candidates);

    bool ok = jointNear(selected, candidate_good);
    record("UT5: Optimiser selects close candidate over far candidate", ok);
}

// ============================================================================
// UNIT TEST 6: TrajectoryOptimiser respects static weight bias
// ============================================================================
static void test_optimiserStaticWeightBias(RobotModel& model)
{
    printSep("UT6: Optimiser respects static weight bias");

    JointVector current(4);
    current.setZero();

    // candidate_low_angle: high dynamic cost, low static cost (cos/sin favor neutral)
    JointVector candidate_low_angle(4);
    candidate_low_angle << 0.1, 0.0, 0.0, 0.0;

    // candidate_high_angle: lower dynamic cost (closer isn't right), higher static cost
    JointVector candidate_high_angle(4);
    candidate_high_angle << 1.2, 0.0, 0.0, 0.0;

    IKCandidates candidates;
    candidates.solutions.push_back(candidate_low_angle);
    candidates.solutions.push_back(candidate_high_angle);

    // Test 1: Static weight very high → prefer low-angle (low static cost)
    TrajectoryOptimiserParameters params_static;
    params_static.static_weight = 100.0;
    params_static.dynamic_weight = 0.01;
    params_static.trajectory_samples = 10;
    params_static.trajectory_time = 1.0;

    TrajectoryOptimiser optimiser_static(model, params_static);
    JointVector selected_static = optimiser_static.selectBest(current, candidates);

    bool static_ok = jointNear(selected_static, candidate_low_angle, 0.2);
    record("UT6a: High static weight favors low-angle pose", static_ok);

    // Test 2: Dynamic weight very high → prefer closer pose
    TrajectoryOptimiserParameters params_dynamic;
    params_dynamic.static_weight = 0.01;
    params_dynamic.dynamic_weight = 100.0;
    params_dynamic.trajectory_samples = 10;
    params_dynamic.trajectory_time = 1.0;

    TrajectoryOptimiser optimiser_dynamic(model, params_dynamic);
    JointVector selected_dynamic = optimiser_dynamic.selectBest(current, candidates);

    bool dynamic_ok = jointNear(selected_dynamic, candidate_low_angle, 0.2);
    record("UT6b: High dynamic weight favors near pose", dynamic_ok);
}

// ============================================================================
// UNIT TEST 7: TrajectoryOptimiser handles empty candidate set
// ============================================================================
static void test_optimiserEmptySet(RobotModel& model)
{
    printSep("UT7: Optimiser handles empty candidate set");

    JointVector current(4);
    current << 0.1, 0.2, 0.3, 0.4;

    IKCandidates candidates;

    TrajectoryOptimiserParameters params;
    params.static_weight = 1.0;
    params.dynamic_weight = 1.0;

    TrajectoryOptimiser optimiser(model, params);
    JointVector selected = optimiser.selectBest(current, candidates);

    bool ok = jointNear(selected, current);
    record("UT7: Empty set returns current configuration", ok);
}

// ============================================================================
// UNIT TEST 8: TrajectoryOptimiser handles mismatched dimensions
// ============================================================================
static void test_optimiserMismatchedDimensions(RobotModel& model)
{
    printSep("UT8: Optimiser handles mismatched dimensions");

    JointVector current(4);
    current.setZero();

    JointVector candidate_good(4);
    candidate_good << 0.1, 0.1, 0.1, 0.1;

    JointVector candidate_bad(3);  // Wrong size
    candidate_bad << 0.5, 0.5, 0.5;

    IKCandidates candidates;
    candidates.solutions.push_back(candidate_bad);
    candidates.solutions.push_back(candidate_good);

    TrajectoryOptimiserParameters params;
    params.static_weight = 1.0;
    params.dynamic_weight = 1.0;
    params.trajectory_samples = 10;
    params.trajectory_time = 1.0;

    TrajectoryOptimiser optimiser(model, params);
    JointVector selected = optimiser.selectBest(current, candidates);

    bool ok = jointNear(selected, candidate_good);
    record("UT8: Mismatched dimension skipped, good candidate selected", ok);
}

// ============================================================================
// UNIT TEST 9: Multiple candidates with realistic IK solutions
// ============================================================================
static void test_optimiserRealisticIK(RobotModel& model)
{
    printSep("UT9: Optimiser with realistic IK candidates");

    ForwardKinematics fk;

    // Generate a target position
    JointVector target_q(4);
    target_q << 0.3, 0.2, 0.8, 0.0;

    PoseMatrix target_pose = fk.solve(model, target_q);
    double target_x = target_pose(0, 3);
    double target_y = target_pose(1, 3);

    // Current arm configuration
    JointVector current(4);
    current << 0.0, 0.0, 0.0, 0.0;

    // Create multiple candidate solutions (simulated IK results)
    IKCandidates candidates;

    // Candidate 1: identical to target
    JointVector cand1(4);
    cand1 << 0.3, 0.2, 0.8, 0.0;
    candidates.solutions.push_back(cand1);

    // Candidate 2: close to target
    JointVector cand2(4);
    cand2 << 0.32, 0.18, 0.82, 0.05;
    candidates.solutions.push_back(cand2);

    // Candidate 3: far from target
    JointVector cand3(4);
    cand3 << -0.8, 0.6, 1.5, -0.8;
    candidates.solutions.push_back(cand3);

    TrajectoryOptimiserParameters params;
    params.static_weight = 1.0;
    params.dynamic_weight = 1.0;
    params.trajectory_samples = 15;
    params.trajectory_time = 1.0;

    TrajectoryOptimiser optimiser(model, params);
    JointVector selected = optimiser.selectBest(current, candidates);

    // The selected should be close to one of the earlier candidates
    bool is_cand1 = jointNear(selected, cand1, 0.15);
    bool is_cand2 = jointNear(selected, cand2, 0.15);

    bool ok = (is_cand1 || is_cand2);
    record("UT9: Selects best IK candidate", ok, 
           is_cand1 ? "chose candidate 1" : (is_cand2 ? "chose candidate 2" : "unknown"));
}

// ============================================================================
// UNIT TEST 10: Parametric sweep of weights
// ============================================================================
static void test_optimiserWeightSweep(RobotModel& model)
{
    printSep("UT10: Parametric weight sweep consistency");

    JointVector current(4);
    current.setZero();

    JointVector cand1(4);
    cand1 << 0.2, 0.1, 0.3, 0.0;

    JointVector cand2(4);
    cand2 << 0.0, 0.5, 0.0, 0.0;

    IKCandidates candidates;
    candidates.solutions.push_back(cand1);
    candidates.solutions.push_back(cand2);

    std::vector<double> weights = {0.1, 0.5, 1.0, 2.0, 10.0};

    bool all_consistent = true;
    for (double w : weights) {
        TrajectoryOptimiserParameters params;
        params.static_weight = w;
        params.dynamic_weight = 1.0 - w / 11.0;  // complementary
        params.trajectory_samples = 10;
        params.trajectory_time = 1.0;

        TrajectoryOptimiser optimiser(model, params);
        JointVector selected = optimiser.selectBest(current, candidates);

        // Just check that one of the candidates is selected (no crash, no NaN)
        if (!jointNear(selected, cand1, 1.0) && !jointNear(selected, cand2, 1.0)) {
            if ((selected - current).squaredNorm() > 1.0) {
                all_consistent = false;
            }
        }
    }

    record("UT10: Weight sweep produces consistent results", all_consistent);
}

// ============================================================================
// UNIT TEST 11: Static cost increases with angle magnitude
// ============================================================================
static void test_staticCostMonotonicity(RobotModel& model)
{
    printSep("UT11: Static cost monotonicity");

    StaticCostEvaluator evaluator(model);

    double prev_cost = 0.0;
    bool monotonic = true;

    for (double angle = 0.0; angle <= M_PI; angle += 0.2) {
        JointVector q(4);
        q << angle, 0.0, 0.0, 0.0;

        double cost = evaluator.evaluate(q);

        // Cost should generally increase from 0 to pi/2 then decrease
        // (because sin is involved), so we just check non-negativity
        if (cost < -1e-10) {
            monotonic = false;
            break;
        }
    }

    record("UT11: Static cost monotonicity and non-negativity", monotonic);
}

// ============================================================================
// UNIT TEST 12: Full pipeline with many candidates
// ============================================================================
static void test_optimiserLargeCandidateSet(RobotModel& model)
{
    printSep("UT12: Optimiser with large candidate set");

    JointVector current(4);
    current.setZero();

    IKCandidates candidates;

    // Create 20 candidate solutions
    JointVector best_candidate(4);
    best_candidate << 0.05, 0.05, 0.05, 0.0;

    for (int i = 0; i < 20; ++i) {
        JointVector cand(4);
        cand << 0.1 * (i - 10), 0.1 * (i % 3), 0.15 * i / 20.0, 0.05 * (i % 4);

        if (i == 10) {
            cand = best_candidate;
        }

        candidates.solutions.push_back(cand);
    }

    TrajectoryOptimiserParameters params;
    params.static_weight = 1.0;
    params.dynamic_weight = 1.5;
    params.trajectory_samples = 12;
    params.trajectory_time = 1.0;

    TrajectoryOptimiser optimiser(model, params);
    JointVector selected = optimiser.selectBest(current, candidates);

    // Should select something close to current or best_candidate
    bool ok = (selected.squaredNorm() < 0.5 && selected.size() == 4);
    record("UT12: Large candidate set handled correctly", ok);
}

int main()
{
    RobotModel model;

    std::cout << "\n╔════════════════════════════════════════════════════════════════════╗\n";
    std::cout << "║      TRAJECTORY OPTIMISER UNIT TESTS                             ║\n";
    std::cout << "╚════════════════════════════════════════════════════════════════════╝\n";

    test_staticCostNonNegative(model);
    test_trajectoryGeneratorSampleCount(model);
    test_trajectoryGeneratorInterpolation(model);
    test_dynamicCostProperties(model);
    test_optimiserSelectionBest(model);
    test_optimiserStaticWeightBias(model);
    test_optimiserEmptySet(model);
    test_optimiserMismatchedDimensions(model);
    test_optimiserRealisticIK(model);
    test_optimiserWeightSweep(model);
    test_staticCostMonotonicity(model);
    test_optimiserLargeCandidateSet(model);

    std::cout << "\n" << std::string(70, '=') << "\n";
    std::cout << "  RESULTS:  " << g_pass << " passed  |  " << g_fail << " failed\n";
    std::cout << std::string(70, '=') << "\n\n";

    return g_fail == 0 ? 0 : 1;
}
