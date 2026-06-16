#include <iostream>
#include <cmath>
#include <string>

#include "pwm_output.hpp"
#include "robot_model.hpp"
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

static bool near(const JointVector& a, const JointVector& b, double tol = 1e-6)
{
    if (a.size() != b.size()) {
        return false;
    }
    return (a - b).cwiseAbs().maxCoeff() < tol;
}

static void test_positionBoundary()
{
    std::cout << "\n=== PWM Output Test: position boundary ===\n";

    JointVector start = JointVector::Zero(5);
    JointVector goal(5);
    goal << 0.1, 0.2, -0.1, 0.5, 1.0;

    MinimumJerkCoefficients coeffs = computeMinimumJerkCoefficients(start, goal, 1.0);
    JointVector atStart = evaluateMinimumJerkPosition(coeffs, 0.0);
    JointVector atEnd = evaluateMinimumJerkPosition(coeffs, 1.0);

    bool ok = near(atStart, start) && near(atEnd, goal);
    record("start and end positions match", ok);
}

static void test_accelerationBoundary()
{
    std::cout << "\n=== PWM Output Test: acceleration boundary ===\n";

    JointVector start = JointVector::Zero(5);
    JointVector goal(5);
    goal << -0.4, 0.6, 0.2, -0.7, 0.8;

    MinimumJerkCoefficients coeffs = computeMinimumJerkCoefficients(start, goal, 2.0);
    JointVector atStart = evaluateMinimumJerkAcceleration(coeffs, 0.0);
    JointVector atEnd = evaluateMinimumJerkAcceleration(coeffs, 2.0);

    bool ok = near(atStart, JointVector::Zero(5)) && near(atEnd, JointVector::Zero(5));
    record("accelerations start and end at zero", ok);
}

static void test_torqueBoundary()
{
    std::cout << "\n=== PWM Output Test: torque boundary ===\n";

    RobotModel model;
    JointVector start = JointVector::Zero(5);
    JointVector goal(5);
    goal << 0.3, -0.3, 0.1, 0.2, 0.5;

    std::vector<JointVector> torqueProfile = generateTorqueProfile(model, start, goal, 1.0, 20);
    bool ok = near(torqueProfile.front(), JointVector::Zero(5)) && near(torqueProfile.back(), JointVector::Zero(5));
    record("torque profile begins and ends at zero", ok);
}

static void test_extraJointTorqueZero()
{
    std::cout << "\n=== PWM Output Test: extra joint torque zero ===\n";

    RobotModel model;
    JointVector start = JointVector::Zero(5);
    JointVector goal(5);
    goal << 0.5, 0.0, 0.0, 0.0, 1.0;

    std::vector<JointVector> torqueProfile = generateTorqueProfile(model, start, goal, 1.0, 10);
    bool ok = true;
    for (const auto& torque : torqueProfile) {
        if (std::abs(torque(4)) > 1e-9) {
            ok = false;
            break;
        }
    }
    record("fifth joint torque remains zero", ok);
}

int main()
{
    test_positionBoundary();
    test_accelerationBoundary();
    test_torqueBoundary();
    test_extraJointTorqueZero();

    std::cout << "\nRESULTS: " << g_pass << " passed, " << g_fail << " failed\n";
    return g_fail == 0 ? 0 : 1;
}
