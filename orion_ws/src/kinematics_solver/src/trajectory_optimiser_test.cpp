#include <iostream>
#include <iomanip>
#include <cmath>
#include <string>
#include <vector>

#include "forward_kinematics.hpp"
#include "trajectory_optimiser.hpp"
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

static void test_staticCostNonNegative(RobotModel& model)
{
    std::cout << "\n=== Trajectory Optimiser Test: static cost non-negative ===\n";

    StaticCostEvaluator evaluator(model);

    JointVector zero(4);
    zero.setZero();
    JointVector candidate(4);
    candidate << 0.5, 0.3, 1.0, -0.4;

    bool ok = evaluator.evaluate(zero) >= 0.0 && evaluator.evaluate(candidate) >= 0.0;
    record("static cost non-negative", ok);
}

static void test_trajectoryGeneratorSamples(RobotModel& model)
{
    std::cout << "\n=== Trajectory Optimiser Test: sample count ===\n";

    for (int samples : {2, 5, 10}) {
        TrajectoryGenerator gen(model.getDOF(), samples);
        JointVector start(4);
        start.setZero();
        JointVector goal(4);
        goal << 0.5, 0.3, 1.0, -0.4;

        auto trajectory = gen.generate(start, goal);
        record("trajectory sample count " + std::to_string(samples), trajectory.size() == static_cast<size_t>(samples));
    }
}

static void test_trajectoryGeneratorInterpolation(RobotModel& model)
{
    std::cout << "\n=== Trajectory Optimiser Test: interpolation ===\n";

    TrajectoryGenerator gen(model.getDOF(), 11);
    JointVector start(4);
    start.setZero();
    JointVector goal(4);
    goal << 1.0, 2.0, 3.0, 4.0;

    std::vector<JointVector> trajectory = gen.generate(start, goal);
    bool startOk = near(trajectory.front(), start);
    bool endOk = near(trajectory.back(), goal);
    JointVector midpoint = (start + goal) / 2.0;
    bool midOk = near(trajectory[5], midpoint);

    record("path starts at start", startOk);
    record("path ends at goal", endOk);
    record("midpoint is interpolated", midOk);
}

static void test_dynamicCostOrdering(RobotModel& model)
{
    std::cout << "\n=== Trajectory Optimiser Test: dynamic cost ordering ===\n";

    DynamicCostEvaluator evaluator(model);
    JointVector start(4);
    start.setZero();
    JointVector nearGoal(4);
    nearGoal << 0.1, 0.1, 0.1, 0.1;
    JointVector farGoal(4);
    farGoal << 1.0, 1.0, 1.0, 1.0;

    TrajectoryGenerator gen(model.getDOF(), 10);
    auto nearTraj = gen.generate(start, nearGoal);
    auto farTraj = gen.generate(start, farGoal);

    double nearCost = evaluator.evaluate(start, nearGoal, nearTraj, 1.0);
    double farCost = evaluator.evaluate(start, farGoal, farTraj, 1.0);

    bool ok = nearCost >= 0.0 && farCost >= 0.0 && farCost > nearCost;
    record("dynamic cost ordering", ok);
}

static void test_selectBestCandidate(RobotModel& model)
{
    std::cout << "\n=== Trajectory Optimiser Test: select best candidate ===\n";

    JointVector current(4);
    current.setZero();
    JointVector good(4);
    good << 0.05, 0.05, 0.05, 0.05;
    JointVector bad(4);
    bad << 1.0, 1.0, 1.0, 1.0;

    IKCandidates candidates;
    candidates.solutions.push_back(bad);
    candidates.solutions.push_back(good);

    TrajectoryOptimiserParameters params;
    params.static_weight = 1.0;
    params.dynamic_weight = 1.0;
    params.trajectory_samples = 10;
    params.trajectory_time = 1.0;

    TrajectoryOptimiser optimiser(model, params);
    JointVector selected = optimiser.selectBest(current, candidates);
    record("selects best candidate", near(selected, good));
}

static void test_weightBias(RobotModel& model)
{
    std::cout << "\n=== Trajectory Optimiser Test: weight bias ===\n";

    JointVector current(4);
    current.setZero();
    JointVector lowAngle(4);
    lowAngle << 0.1, 0.0, 0.0, 0.0;
    JointVector flexed(4);
    flexed << 1.2, 0.0, 0.0, 0.0;

    IKCandidates candidates;
    candidates.solutions.push_back(lowAngle);
    candidates.solutions.push_back(flexed);

    TrajectoryOptimiserParameters staticParams;
    staticParams.static_weight = 100.0;
    staticParams.dynamic_weight = 0.01;
    staticParams.trajectory_samples = 10;
    staticParams.trajectory_time = 1.0;

    TrajectoryOptimiser staticOptimiser(model, staticParams);
    JointVector staticSelected = staticOptimiser.selectBest(current, candidates);
    record("static bias selects low torque", near(staticSelected, flexed, 0.2));

    TrajectoryOptimiserParameters dynamicParams;
    dynamicParams.static_weight = 0.01;
    dynamicParams.dynamic_weight = 100.0;
    dynamicParams.trajectory_samples = 10;
    dynamicParams.trajectory_time = 1.0;

    TrajectoryOptimiser dynamicOptimiser(model, dynamicParams);
    JointVector dynamicSelected = dynamicOptimiser.selectBest(current, candidates);
    record("dynamic bias selects close pose", near(dynamicSelected, lowAngle, 0.2));
}

static void test_emptyCandidates(RobotModel& model)
{
    std::cout << "\n=== Trajectory Optimiser Test: empty candidates ===\n";

    JointVector current(4);
    current << 0.1, 0.2, 0.3, 0.4;
    IKCandidates candidates;
    TrajectoryOptimiserParameters params;
    params.static_weight = 1.0;
    params.dynamic_weight = 1.0;

    TrajectoryOptimiser optimiser(model, params);
    JointVector selected = optimiser.selectBest(current, candidates);
    record("empty candidate set returns current", near(selected, current));
}

static void test_mismatchedCandidateSize(RobotModel& model)
{
    std::cout << "\n=== Trajectory Optimiser Test: mismatched candidate size ===\n";

    JointVector current(4);
    current.setZero();
    JointVector good(4);
    good << 0.1, 0.1, 0.1, 0.1;
    JointVector bad(3);
    bad << 0.5, 0.5, 0.5;

    IKCandidates candidates;
    candidates.solutions.push_back(bad);
    candidates.solutions.push_back(good);

    TrajectoryOptimiserParameters params;
    params.static_weight = 1.0;
    params.dynamic_weight = 1.0;
    params.trajectory_samples = 10;
    params.trajectory_time = 1.0;

    TrajectoryOptimiser optimiser(model, params);
    JointVector selected = optimiser.selectBest(current, candidates);
    record("skips invalid candidate sizes", near(selected, good));
}

static void test_realisticCandidateSelection(RobotModel& model)
{
    std::cout << "\n=== Trajectory Optimiser Test: realistic candidate selection ===\n";

    ForwardKinematics fk;
    JointVector target(4);
    target << 0.3, 0.2, 0.8, 0.0;
    PoseMatrix pose = fk.solve(model, target);
    Position targetPose(3);
    targetPose << pose(0, 3), 0.0, pose(1, 3);

    JointVector current(4);
    current.setZero();

    IKCandidates candidates;
    candidates.solutions.push_back(target);
    candidates.solutions.push_back((JointVector(4) << 0.32, 0.18, 0.82, 0.05).finished());
    candidates.solutions.push_back((JointVector(4) << -0.8, 0.6, 1.5, -0.8).finished());

    TrajectoryOptimiserParameters params;
    params.static_weight = 1.0;
    params.dynamic_weight = 1.0;
    params.trajectory_samples = 15;
    params.trajectory_time = 1.0;

    TrajectoryOptimiser optimiser(model, params);
    JointVector selected = optimiser.selectBest(current, candidates);

    bool ok = near(selected, target, 0.15) || near(selected, (JointVector(4) << 0.32, 0.18, 0.82, 0.05).finished(), 0.15);
    record("selects a valid realistic candidate", ok);
}

int main()
{
    RobotModel model;

    test_staticCostNonNegative(model);
    test_trajectoryGeneratorSamples(model);
    test_trajectoryGeneratorInterpolation(model);
    test_dynamicCostOrdering(model);
    test_selectBestCandidate(model);
    test_weightBias(model);
    test_emptyCandidates(model);
    test_mismatchedCandidateSize(model);
    test_realisticCandidateSelection(model);

    std::cout << "\nRESULTS: " << g_pass << " passed, " << g_fail << " failed\n";
    return g_fail == 0 ? 0 : 1;
}
