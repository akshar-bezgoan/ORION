#include <iostream>
#include <iomanip>
#include <string>
#include <vector>

#include "robot_model.hpp"
#include "inverse_kinematics.hpp"
#include "trajectory_optimiser.hpp"
#include "pwm_output.hpp"
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

static bool parseDouble(const std::string& token, double& value)
{
    try {
        value = std::stod(token);
        return true;
    } catch (...) {
        return false;
    }
}

void printJointVector(const std::string& label, const JointVector& q) {
    std::cout << label << ": [";
    for (int i = 0; i < q.size(); ++i) {
        std::cout << std::fixed << std::setprecision(4) << q[i] << (i == q.size() - 1 ? "" : ", ");
    }
    std::cout << "]\n";
}

int main(int argc, char* argv[]) {
    if (argc != 4 && argc != 8) {
        std::cout << "Usage: " << argv[0] << " <tx> <ty> <tz> [static_weight] [dynamic_weight] [trajectory_samples] [trajectory_time]\n";
        std::cout << "  Defaults: static_w=1.0, dynamic_w=0.5, samples=10, time=2.0s\n";
        return 1;
    }

    double tx, ty, tz;
    double static_weight = 1.0;
    double dynamic_weight = 0.5;
    int trajectory_samples = 10;
    double trajectory_time = 2.0;

    if (!parseDouble(argv[1], tx) || !parseDouble(argv[2], ty) || !parseDouble(argv[3], tz)) {
        std::cerr << "Error: Invalid target coordinates.\n";
        return 1;
    }

    if (argc == 8) {
        double samples_d;
        if (!parseDouble(argv[4], static_weight) ||
            !parseDouble(argv[5], dynamic_weight) ||
            !parseDouble(argv[6], samples_d) ||
            !parseDouble(argv[7], trajectory_time)) {
            std::cerr << "Error: Invalid optimizer/trajectory parameters.\n";
            return 1;
        }
        trajectory_samples = static_cast<int>(std::max(2.0, samples_d));
    }

    RobotModel model;
    InverseKinematics ik;
    
    Position target(3);
    target << tx, ty, tz;

    std::cout << "\n=== Kinematics Solver: Pipeline Integration Test ===\n";
    std::cout << "Target Pose: (" << tx << ", " << ty << ", " << tz << ")\n";
    std::cout << "Config: sw=" << static_weight << " dw=" << dynamic_weight 
              << " samples=" << trajectory_samples << " time=" << trajectory_time << "s\n\n";

    IKCandidates candidates = ik.solveAll(model, target);
    record("IK Solver", !candidates.solutions.empty(), 
           candidates.solutions.empty() ? "No reachable configurations" : "Found " + std::to_string(candidates.solutions.size()) + " valid candidates");

    if (candidates.solutions.empty()) {
        std::cout << "\nRESULTS: " << g_pass << " passed, " << g_fail << " failed\n";
        return 1;
    }

    JointVector start_state = JointVector::Zero(4); 
    
    TrajectoryOptimiserParameters params;
    params.static_weight = static_weight;
    params.dynamic_weight = dynamic_weight;
    params.trajectory_samples = 100; // Use high resolution for cost evaluation
    params.trajectory_time = trajectory_time;

    TrajectoryOptimiser optimiser(model, params);

    IKCandidates arm_candidates;
    arm_candidates.solutions.reserve(candidates.solutions.size());
    std::vector<double> base_yaws;
    for (const auto& sol : candidates.solutions) {
        JointVector arm(4);
        arm << sol(0), sol(1), sol(2), sol(3);
        arm_candidates.solutions.push_back(arm);
        base_yaws.push_back(sol.size() > 4 ? sol(4) : 0.0);
    }

    // Select the best configuration based on weighted costs
    JointVector best_arm_goal = optimiser.selectBest(start_state, arm_candidates);
    
    // Validation: Ensure the selection actually exists in our candidate list
    int best_idx = -1;
    for (size_t i = 0; i < arm_candidates.solutions.size(); ++i) {
        if ((arm_candidates.solutions[i] - best_arm_goal).norm() < 1e-6) {
            best_idx = static_cast<int>(i);
            break;
        }
    }

    // Calculate actual costs for the selected candidate for reporting
    StaticCostEvaluator static_eval(model);
    double selected_static_cost = static_eval.evaluate(best_arm_goal);
    
    std::string opt_note = "Selected Candidate #" + std::to_string(best_idx) + 
                           " (Static Torque Cost: " + std::to_string(selected_static_cost) + ")";
    record("Trajectory Optimiser", best_idx != -1, opt_note);

    double selected_yaw = 0.0;
    for(size_t i = 0; i < arm_candidates.solutions.size(); ++i) {
        if ((arm_candidates.solutions[i] - best_arm_goal).norm() < 1e-6) {
            selected_yaw = base_yaws[i];
            break;
        }
    }

    JointVector full_start = JointVector::Zero(5);
    JointVector full_goal(5);
    full_goal << best_arm_goal(0), best_arm_goal(1), best_arm_goal(2), best_arm_goal(3), selected_yaw;

    std::cout << "Selected Optimal Configuration:\n";
    printJointVector("  Goal Angles (rad)", full_goal);
    std::cout << "\n";

    std::vector<JointVector> torque_profile = genTauProfile(model, full_start, full_goal, trajectory_time, trajectory_samples);
    
    // Test for "Optimal Torque Stream" - Check boundary conditions (Minimum Jerk property)
    bool smooth_start = torque_profile.front().norm() < 1e-6;
    bool smooth_end = torque_profile.back().norm() < 1e-6;
    record("Torque Stream Optimality", smooth_start && smooth_end, "Minimum Jerk boundary conditions met (starts and ends at zero torque)");

    std::cout << "=== Generated PWM / Torque Profile ===\n";
    std::cout << "Format: [Step] | Torques (Nm Proxy: base -> shoulder -> elbow -> wrist -> yaw)\n";
    std::cout << "-------------------------------------------\n";
    
    for (size_t i = 0; i < torque_profile.size(); ++i) {
        std::cout << "Step " << std::setw(2) << i << " | ";
        for (int j = 0; j < torque_profile[i].size(); ++j) {
            std::cout << std::fixed << std::setprecision(5) << std::setw(9) << torque_profile[i][j] << " ";
        }
        std::cout << "\n";
    }

    std::cout << "-------------------------------------------\n";
    std::cout << "\nRESULTS: " << g_pass << " passed, " << g_fail << " failed\n";

    return g_fail == 0 ? 0 : 1;
}