#include "robot/RobotArm.h"
#include <raymath.h>
#include <algorithm>
#include <sstream>

namespace robot {

static inline double clampd(double v, double lo, double hi) {
    return std::max(lo, std::min(hi, v));
}

RobotArm::RobotArm(const LinkParams& l1, const LinkParams& l2)
: link1_(l1), link2_(l2) {
    link1_.RecomputeInertia();
    link2_.RecomputeInertia();
}


IKResult RobotArm::SolveIK(const Vector3& target, bool elbowUp) const {
    IKResult out;

    // Enforce workspace rule: z must be non-negative
    if (target.z < 0.0f) {
        out.reachable = false;
        out.message = "Invalid target: z must be >= 0";
        return out;
    }

    const double x = target.x;
    const double y = target.y;
    const double z = target.z;

    const double d = std::sqrt(x*x + y*y + z*z);
    const double rmin = MinReach();
    const double rmax = MaxReach();

    if (d < rmin - 1e-9 || d > rmax + 1e-9) {
        out.reachable = false;
        std::ostringstream oss;
        oss << "Target radius |p|=" << d
            << " is outside [" << rmin << ", " << rmax << "]";
        out.message = oss.str();
        return out;
    }

    // Base yaw: angle in XY plane
    double q0 = 0.0;
    if (std::fabs(x) > 1e-12 || std::fabs(y) > 1e-12) {
        q0 = std::atan2(y, x);
    }

    // Reduce to planar IK in the (r, z) plane, where r = sqrt(x^2+y^2)
    const double r = std::sqrt(x*x + y*y);
    const double L1 = link1_.length_m;
    const double L2 = link2_.length_m;

    // Law of cosines for elbow angle
    double c2 = (r*r + z*z - L1*L1 - L2*L2) / (2.0 * L1 * L2);
    c2 = clampd(c2, -1.0, 1.0);

    double q2 = std::acos(c2);
    if (!elbowUp) q2 = -q2; // choose elbow-down by default (negative bend)

    // Shoulder angle
    const double s2 = std::sin(q2);
    const double k1 = L1 + L2 * std::cos(q2);
    const double k2 = L2 * s2;

    // q1 measured relative to XY plane (horizontal): atan2(z, r) gives elevation
    const double q1 = std::atan2(z, r) - std::atan2(k2, k1);

    out.reachable = true;
    out.q.q0_yaw   = (float)q0;
    out.q.q1_pitch = (float)q1;
    out.q.q2_pitch = (float)q2;
    out.message = "OK";
    return out;
}

FKResult RobotArm::ForwardKinematics(const JointAngles& q) const {
    FKResult fk;
    fk.base = {0,0,0};
    fk.joint1 = fk.base;

    const float L1 = link1_.length_m;
    const float L2 = link2_.length_m;

    // Unit vectors:
    // u = radial direction in XY defined by yaw
    // k = +Z
    const float cy = std::cos(q.q0_yaw);
    const float sy = std::sin(q.q0_yaw);
    Vector3 u{cy, sy, 0.0f};
    Vector3 k{0.0f, 0.0f, 1.0f};

    // Link1 end (elbow)
    Vector3 p1 = Vector3Add(
        Vector3Scale(u, L1 * std::cos(q.q1_pitch)),
        Vector3Scale(k, L1 * std::sin(q.q1_pitch))
    );

    // Link2 end (end effector)
    const float a = q.q1_pitch + q.q2_pitch;
    Vector3 p2 = Vector3Add(
        p1,
        Vector3Add(
            Vector3Scale(u, L2 * std::cos(a)),
            Vector3Scale(k, L2 * std::sin(a))
        )
    );

    fk.joint2 = p1;
    fk.ee = p2;
    return fk;
}

} // namespace robot
