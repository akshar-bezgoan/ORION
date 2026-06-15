#include "robot_model.hpp"

RobotModel::RobotModel()
{
    shoulder_h_   = 0.050;  
    upper_arm_l_  = 0.220;  
    forearm_l_    = 0.145;  
    gripper_l_    = 0.055;  

    
    shoulder_m_   = 0.300;  
    upper_arm_m_  = 0.308;  
    forearm_m_    = 0.199;  
    gripper_m_    = 0.120;  

    joint_limits_.resize(DOF);

    
    joint_limits_[0] = {-1.58, 0.79};
    joint_limits_[1] = {-0.25, 1.57};  
    joint_limits_[2] = { 0.00, 2.70};  
    joint_limits_[3] = {-1.57, 1.57}; 
}

int RobotModel::getDOF() const {
    return DOF;
}

double RobotModel::getLinkLength(int idx) const {
    switch (idx) {
        case 0: return shoulder_h_;
        case 1: return upper_arm_l_;
        case 2: return forearm_l_;
        case 3: return gripper_l_;
        default: return 0.0;
    }
}

double RobotModel::getLinkMass(int idx) const {
    switch (idx) {
        case 0: return shoulder_m_;
        case 1: return upper_arm_m_;
        case 2: return forearm_m_;
        case 3: return gripper_m_;
        default: return 0.0;
    }
}

bool RobotModel::isWithinLimits(const Eigen::Vector4d& q) const {
    for (int i = 0; i < DOF; i++) {
        if (q[i] < joint_limits_[i].min || q[i] > joint_limits_[i].max) {
            return false;
        }
    }
    return true;
}
