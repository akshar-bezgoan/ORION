#pragma once

#include <Eigen/Dense>
#include <vector>

using JointVector = Eigen::VectorXd;

using PoseMatrix = Eigen::Matrix4d;

struct IKResult {
    JointVector joints;   
    bool success;
    double error;
};

struct IKCandidates {
    std::vector<JointVector> solutions;
};

