#include <iostream>
#include <iomanip>
#include <string>
#include <vector>

#include "inverse_kinematics.hpp"
#include "robot_model.hpp"
#include "trajectory_optimiser.hpp"
#include "types.hpp"

static void printUsage(const std::string& programName)
{
    std::cout << "Usage: " << programName << " <tx> <ty> <tz> [static_weight] [dynamic_weight] [trajectory_samples] [trajectory_time]\n";
    std::cout << "  tx ty tz             : target end effector coordinates in meters\n";
    std::cout << "  static_weight         : cost weight for static torque (default 1.0)\n";
    std::cout << "  dynamic_weight        : cost weight for motion cost (default 0.05)\n";
    std::cout << "  trajectory_samples    : number of samples for trajectory generation (default 1000)\n";
    std::cout << "  trajectory_time       : duration used for dynamic cost (default 1.0)\n";
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

int main(int argc, char* argv[])
{
    if (argc != 4 && argc != 8) {
        printUsage(argv[0]);
        return 1;
    }

    double tx = 0.0;
    double ty = 0.0;
    double tz = 0.0;
    double static_weight = 1.0;
    double dynamic_weight = 0.05;
    int trajectory_samples = 1000;
    double trajectory_time = 1.0;

    if (!parseDouble(argv[1], tx) || !parseDouble(argv[2], ty) || !parseDouble(argv[3], tz)) {
        std::cerr << "Error: invalid target coordinates.\n";
        printUsage(argv[0]);
        return 1;
    }

    if (argc == 8) {
        double trajectory_samples_d = 0.0;
        if (!parseDouble(argv[4], static_weight) ||
            !parseDouble(argv[5], dynamic_weight) ||
            !parseDouble(argv[6], trajectory_samples_d) ||
            !parseDouble(argv[7], trajectory_time)) {
            std::cerr << "Error: invalid optimiser parameters.\n";
            printUsage(argv[0]);
            return 1;
        }
        trajectory_samples = static_cast<int>(std::max(2.0, std::floor(trajectory_samples_d)));
    }

    RobotModel model;
    InverseKinematics ik;
    Position target(3);
    target << tx, ty, tz;

    IKCandidates candidates = ik.solveAll(model, target);
    if (candidates.solutions.empty()) {
        std::cout << "No IK solutions found for target (" << tx << ", " << ty << ", " << tz << ").\n";
        return 2;
    }

    std::cout << "Found " << candidates.solutions.size() << " IK solutions.\n";

    JointVector current(4);
    current.setZero();

    TrajectoryOptimiserParameters params;
    params.static_weight = static_weight;
    params.dynamic_weight = dynamic_weight;
    params.trajectory_samples = trajectory_samples;
    params.trajectory_time = trajectory_time;

    TrajectoryOptimiser optimiser(model, params);

    IKCandidates armCandidates;
    armCandidates.solutions.reserve(candidates.solutions.size());
    std::vector<double> baseYaws;
    baseYaws.reserve(candidates.solutions.size());

    for (const auto& candidate : candidates.solutions) {
        if (candidate.size() == 5) {
            JointVector arm(4);
            arm << candidate(0), candidate(1), candidate(2), candidate(3);
            armCandidates.solutions.push_back(arm);
            baseYaws.push_back(candidate(4));
        } else if (candidate.size() == 4) {
            armCandidates.solutions.push_back(candidate);
            baseYaws.push_back(0.0);
        }
    }

    if (armCandidates.solutions.empty()) {
        std::cout << "No arm solutions to optimise after filtering.\n";
        return 3;
    }

    JointVector selected = optimiser.selectBest(current, armCandidates);

    int selectedIndex = -1;
    for (size_t i = 0; i < armCandidates.solutions.size(); ++i) {
        if ((armCandidates.solutions[i] - selected).cwiseAbs().maxCoeff() < 1e-6) {
            selectedIndex = static_cast<int>(i);
            break;
        }
    }

    std::cout << std::fixed << std::setprecision(6);
    std::cout << "Selected optimal IK solution:\n";
    std::cout << "  q0=" << selected(0) << " rad, q1=" << selected(1) << " rad, ";
    std::cout << "q2=" << selected(2) << " rad, q3=" << selected(3) << " rad\n";

    if (selectedIndex >= 0) {
        std::cout << "  base_yaw=" << baseYaws[selectedIndex] << " rad\n";
    } else {
        std::cout << "  base_yaw=unknown (candidate match failed)\n";
    }

    std::cout << "  static_weight=" << static_weight
              << " dynamic_weight=" << dynamic_weight
              << " trajectory_samples=" << trajectory_samples
              << " trajectory_time=" << trajectory_time << "\n";

    return 0;
}
