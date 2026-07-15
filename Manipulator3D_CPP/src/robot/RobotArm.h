#pragma once
#include <raylib.h>
#include <cmath>
#include <string>

namespace robot {

struct LinkParams {
    float length_m = 2.5f;
    float mass_kg  = 1.0f;

    // Approximations for a uniform rod (about center, axis ⟂ to rod):
    // I_cm = (1/12) m L^2
    // I_joint (about joint at one end) = (1/3) m L^2
    float inertia_cm = 0.0f;
    float inertia_joint = 0.0f;

    void RecomputeInertia() {
        inertia_cm = (1.0f / 12.0f) * mass_kg * length_m * length_m;
        inertia_joint = (1.0f / 3.0f) * mass_kg * length_m * length_m;
    }
};

struct JointAngles {
    float q0_yaw   = 0.0f; // rad
    float q1_pitch = 0.0f; // rad (relative to XY plane)
    float q2_pitch = 0.0f; // rad (elbow relative, in same plane)
};

struct FKResult {
    Vector3 base{0,0,0};
    Vector3 joint1{0,0,0}; // shoulder location (same as base here)
    Vector3 joint2{0,0,0}; // elbow position
    Vector3 ee{0,0,0};     // end effector position
};

struct IKResult {
    bool reachable = false;
    JointAngles q{};
    std::string message;
};

class RobotArm {
public:
    RobotArm(const LinkParams& l1, const LinkParams& l2);

    float L1() const { return link1_.length_m; }
    float L2() const { return link2_.length_m; }

    float MaxReach() const { return link1_.length_m + link2_.length_m; }
    float MinReach() const { return std::fabs(link1_.length_m - link2_.length_m); }

    const LinkParams& Link1() const { return link1_; }
    const LinkParams& Link2() const { return link2_; }

    // Inverse kinematics for target in 3D
    // elbowUp: chooses elbow-up vs elbow-down branch for q2 sign
    IKResult SolveIK(const Vector3& target, bool elbowUp = false) const;

    // Forward kinematics (positions of joints and end effector)
    FKResult ForwardKinematics(const JointAngles& q) const;

private:
    LinkParams link1_;
    LinkParams link2_;
};

} // namespace robot
