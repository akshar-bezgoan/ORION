#include <iostream>
#include "inverse_kinematics.hpp"

void PrintIK(const IKCandidates& c);

int main(){
    RobotModel model;
    InverseKinematics ik;
    Position target;
    target << 0,0,0.47;
    IKCandidates c = ik.solveAll(model, target);
    std::cout << "Returned " << c.solutions.size() << "solutions\n";
    PrintIK(c);
}

void PrintIK(const IKCandidates& c)
{
    std::cout << "Number of solutions: "
              << c.solutions.size()
              << std::endl;

    for (size_t i = 0; i < c.solutions.size(); ++i) {
        std::cout << "IK Solution " << i << ": "
                  << c.solutions[i].transpose()
                  << std::endl;
    }
}