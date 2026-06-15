#pragma once
#include <Eigen/Dense>
#include <vector>

using JointVector = Eigen::VectorXd;

using PoseMatrix = Eigen::Matrix4d;

using Position = Eigen::Vector3d;

struct IKResult {
    JointVector joints;
    bool success;
    double error;
};

struct IKCandidates {
    std::vector<JointVector> solutions;
};

struct TrajectoryOptimiserParameters {
    double static_weight = 1.0;
    double dynamic_weight = 0.05; 
    int trajectory_samples = 1000;
    double trajectory_time = 1.0;
};