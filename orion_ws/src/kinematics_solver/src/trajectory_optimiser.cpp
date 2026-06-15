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
    const double gravity = 9.81;
    const int dof = model_.getDOF();
    const int size = static_cast<int>(q.size());

    if (size < dof) {
        return std::numeric_limits<double>::infinity();
    }

    std::vector<double> joint_x(dof + 1, 0.0);
    std::vector<double> joint_y(dof + 1, 0.0);
    std::vector<double> com_x(dof, 0.0);

    double angle = 0.0;
    for (int i = 0; i < dof; ++i) {
        angle += q[i];
        double link_length = model_.getLinkLength(i);
        double cx = joint_x[i] + 0.5 * link_length * std::cos(angle);
        double cy = joint_y[i] + 0.5 * link_length * std::sin(angle);
        com_x[i] = cx;

        joint_x[i + 1] = joint_x[i] + link_length * std::cos(angle);
        joint_y[i + 1] = joint_y[i] + link_length * std::sin(angle);
    }

    double cost = 0.0;
    for (int joint = 0; joint < dof; ++joint) {
        double torque = 0.0;
        for (int link = joint; link < dof; ++link) {
            double mass = model_.getLinkMass(link);
            double lever_arm = com_x[link] - joint_x[joint];
            torque += mass * gravity * lever_arm;
        }
        cost += square(torque);
    }

    return cost;
}

static double computeDynamicTorqueCost(const RobotModel& model,
                                       const std::vector<JointVector>& trajectory,
                                       double trajectory_time) {
    if (trajectory.size() < 3) {
        return 0.0;
    }

    const int dof = model.getDOF();
    const double dt = trajectory_time / static_cast<double>(trajectory.size() - 1);
    JointVector prev_velocity = JointVector::Zero(dof);
    JointVector current_velocity(dof);

    double cost = 0.0;
    for (size_t step = 1; step < trajectory.size(); ++step) {
        const JointVector& prev = trajectory[step - 1];
        const JointVector& curr = trajectory[step];
        current_velocity = (curr - prev) / dt;
        JointVector acceleration = (current_velocity - prev_velocity) / dt;

        double step_torque_sq = 0.0;
        for (int joint = 0; joint < dof; ++joint) {
            double torque_proxy = 0.0;
            for (int link = joint; link < dof; ++link) {
                double mass = model.getLinkMass(link);
                double length = model.getLinkLength(link);
                torque_proxy += std::abs(acceleration[joint]) * mass * length;
            }
            step_torque_sq += square(torque_proxy);
        }

        cost += step_torque_sq;
        prev_velocity = current_velocity;
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

    const int dof = model_.getDOF();
    double dt = trajectory_time / static_cast<double>(trajectory.size() - 1);
    JointVector previous_velocity = JointVector::Zero(dof);

    double cost = 0.0;
    for (size_t step = 1; step < trajectory.size(); ++step) {
        const JointVector& prev = trajectory[step - 1];
        const JointVector& curr = trajectory[step];

        JointVector velocity = (curr - prev) / dt;
        JointVector acceleration = (velocity - previous_velocity) / dt;

        double step_torque_sq = 0.0;
        for (int joint = 0; joint < dof; ++joint) {
            double torque_proxy = 0.0;
            for (int link = joint; link < dof; ++link) {
                double mass = model_.getLinkMass(link);
                double length = model_.getLinkLength(link);
                torque_proxy += std::abs(acceleration[joint]) * mass * length;
            }
            step_torque_sq += square(torque_proxy);
        }

        cost += step_torque_sq;
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

    std::vector<double> static_costs;
    std::vector<double> dynamic_costs;
    static_costs.reserve(candidates.solutions.size());
    dynamic_costs.reserve(candidates.solutions.size());

    for (const auto& candidate : candidates.solutions) {
        if (candidate.size() != current.size()) {
            static_costs.push_back(std::numeric_limits<double>::infinity());
            dynamic_costs.push_back(std::numeric_limits<double>::infinity());
            continue;
        }

        double static_cost = static_evaluator_.evaluate(candidate);
        std::vector<JointVector> trajectory = trajectory_generator_.generate(current, candidate);
        double dynamic_cost = dynamic_evaluator_.evaluate(current,
                                                         candidate,
                                                         trajectory,
                                                         params_.trajectory_time);
        static_costs.push_back(static_cost);
        dynamic_costs.push_back(dynamic_cost);
    }

    double max_static = 0.0;
    double max_dynamic = 0.0;
    for (double c : static_costs) {
        if (c > max_static && std::isfinite(c)) max_static = c;
    }
    for (double c : dynamic_costs) {
        if (c > max_dynamic && std::isfinite(c)) max_dynamic = c;
    }

    for (size_t i = 0; i < candidates.solutions.size(); ++i) {
        const auto& candidate = candidates.solutions[i];
        if (candidate.size() != current.size()) {
            continue;
        }

        double norm_static = (max_static > 0.0) ? static_costs[i] / max_static : 0.0;
        double norm_dynamic = (max_dynamic > 0.0) ? dynamic_costs[i] / max_dynamic : 0.0;
        double total_cost = params_.static_weight * norm_static + params_.dynamic_weight * norm_dynamic;

        if (total_cost < best_cost) {
            best_cost = total_cost;
            best_solution = candidate;
        }
    }

    return best_solution;
}

JointVector TrajectoryOptimiser::selectLowestTorque(const IKCandidates& candidates) const {
    JointVector best_solution = JointVector::Zero(model_.getDOF());
    double best_cost = std::numeric_limits<double>::infinity();

    if (candidates.solutions.empty()) {
        return best_solution;
    }

    for (const auto& candidate : candidates.solutions) {
        if (candidate.size() != model_.getDOF()) {
            continue;
        }

        double static_cost = static_evaluator_.evaluate(candidate);

        if (static_cost < best_cost) {
            best_cost = static_cost;
            best_solution = candidate;
        }
    }

    return best_solution;
}
