#include "trajectory_optimiser.hpp"

#include <cmath>
#include <limits>
#include <algorithm>

StaticCostEvaluator::StaticCostEvaluator(const RobotModel& model)
    : model_(model) {}

static double square(double value) {
    return value * value;
}

double StaticCostEvaluator::evaluate(const JointVector& q) const {
    double cost = 0.0;
    int dof = model_.getDOF();
    int size = static_cast<int>(q.size());

    for (int i = 0; i < dof && i < size; ++i) {
        double leverage = 1.0 + 0.35 * i;
        double mass_proxy = 1.0 + 0.45 * i;
        double gravity_proxy = leverage * mass_proxy * std::abs(std::sin(q[i]));
        cost += square(gravity_proxy);
    }

    return cost;
}

TrajectoryGenerator::TrajectoryGenerator(int dof, int samples)
    : dof_(dof), samples_(std::max(2, samples)) {}

std::vector<JointVector> TrajectoryGenerator::generate(const JointVector& start, const JointVector& goal) const {
    std::vector<JointVector> trajectory;
    trajectory.reserve(samples_);

    for (int step = 0; step < samples_; ++step) {
        double alpha = static_cast<double>(step) / (samples_ - 1);
        JointVector point(dof_);
        for (int i = 0; i < dof_; ++i) {
            point[i] = start[i] + alpha * (goal[i] - start[i]);
        }
        trajectory.push_back(point);
    }

    return trajectory;
}

DynamicCostEvaluator::DynamicCostEvaluator(const RobotModel& model)
    : model_(model) {}

double DynamicCostEvaluator::evaluate(const JointVector& start,
                                      const JointVector& goal,
                                      const std::vector<JointVector>& trajectory,
                                      double trajectory_time) const {
    if (trajectory.size() < 2) {
        return 0.0;
    }

    double cost = 0.0;
    double dt = trajectory_time / static_cast<double>(trajectory.size() - 1);
    JointVector previous_velocity = JointVector::Zero(model_.getDOF());

    for (size_t step = 1; step < trajectory.size(); ++step) {
        const JointVector& prev = trajectory[step - 1];
        const JointVector& curr = trajectory[step];

        JointVector velocity = (curr - prev) / dt;
        JointVector acceleration = (velocity - previous_velocity) / dt;

        cost += velocity.squaredNorm();
        cost += acceleration.squaredNorm();
        cost += 0.1 * curr.squaredNorm();

        previous_velocity = velocity;
    }

    return cost;
}

TrajectoryOptimiser::TrajectoryOptimiser(const RobotModel& model,
                                         const TrajectoryOptimiserParameters& params)
    : model_(model),
      params_(params),
      static_evaluator_(model),
      trajectory_generator_(model.getDOF(), params_.trajectory_samples),
      dynamic_evaluator_(model) {}

JointVector TrajectoryOptimiser::selectBest(const JointVector& current,
                                            const IKCandidates& candidates) const {
    JointVector best_solution = current;
    double best_cost = std::numeric_limits<double>::infinity();

    if (candidates.solutions.empty()) {
        return current;
    }

    for (const auto& candidate : candidates.solutions) {
        if (candidate.size() != current.size()) {
            continue;
        }

        std::vector<JointVector> trajectory = trajectory_generator_.generate(current, candidate);
        double static_cost = static_evaluator_.evaluate(candidate);
        double dynamic_cost = dynamic_evaluator_.evaluate(current,
                                                         candidate,
                                                         trajectory,
                                                         params_.trajectory_time);
        double total_cost = params_.static_weight * static_cost +
                            params_.dynamic_weight * dynamic_cost;

        if (total_cost < best_cost) {
            best_cost = total_cost;
            best_solution = candidate;
        }
    }

    return best_solution;
}