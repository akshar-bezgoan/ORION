#pragma once

#include "robot_model.hpp"
#include "types.hpp"
#include <vector>

struct IKSolution {
    JointVector joints;
};


class StaticCostEvaluator {
public:
    explicit StaticCostEvaluator(const RobotModel& model);
    double evaluate(const JointVector& q) const;

private:
    const RobotModel& model_;
};

class TrajectoryGenerator {
public:
    explicit TrajectoryGenerator(int dof, int samples);
    std::vector<JointVector> generate(const JointVector& start,
                                      const JointVector& goal) const;

private:
    int dof_;
    int samples_;
};

class DynamicCostEvaluator {
public:
    explicit DynamicCostEvaluator(const RobotModel& model);
    double evaluate(const JointVector& start,
                    const JointVector& goal,
                    const std::vector<JointVector>& trajectory,
                    double trajectory_time) const;

private:
    const RobotModel& model_;
};

class TrajectoryOptimiser {
public:
    explicit TrajectoryOptimiser(const RobotModel& model,
                                 const TrajectoryOptimiserParameters& params = {});
    JointVector selectBest(const JointVector& current,
                           const IKCandidates& candidates) const;
    JointVector selectLowestTorque(const IKCandidates& candidates) const;

private:
    const RobotModel& model_;
    TrajectoryOptimiserParameters params_;
    StaticCostEvaluator static_evaluator_;
    TrajectoryGenerator trajectory_generator_;
    DynamicCostEvaluator dynamic_evaluator_;
};