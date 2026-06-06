#pragma once

#include <Eigen/Dense>
#include <vector>

class RobotModel {
public:
    RobotModel();

    static constexpr int DOF = 4;

    double shoulder_mount_x;  
    double shoulder_h;        
    double upper_arm_l;       
    double forearm_l;       
    double gripper_l;         
    double gripper_tcp;       

    double shoulder_m;       
    double upper_arm_m;      
    double forearm_m;        
    double gripper_m;        

    struct JointLimit {
        double min;
        double max;
    };

    std::vector<JointLimit> joint_limits;  

    struct JointDynamics {
        double damping;
        double friction;
    };

    std::vector<JointDynamics> joint_dynamics;
    double arm_effort;       
    double arm_vel;           

    bool isWithinLimits(const Eigen::Vector4d& q) const;
    int  getDOF() const;
};
