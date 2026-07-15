module Robot

export Vec3, v3, norm3,
       LinkParams, JointAngles, FKResult, IKResult,
       RobotArm, min_reach, max_reach,
       solve_ik, forward_kinematics

using Printf

# -----------------------------
# Basic 3D vector (standalone)
# -----------------------------

struct Vec3
    x::Float32
    y::Float32
    z::Float32
end

v3(x, y, z) = Vec3(Float32(x), Float32(y), Float32(z))

Base.:+(a::Vec3, b::Vec3) = Vec3(a.x + b.x, a.y + b.y, a.z + b.z)
Base.:-(a::Vec3, b::Vec3) = Vec3(a.x - b.x, a.y - b.y, a.z - b.z)
Base.:*(s::Real, v::Vec3) = Vec3(Float32(s) * v.x, Float32(s) * v.y, Float32(s) * v.z)
Base.:*(v::Vec3, s::Real) = s * v

function norm3(v::Vec3)::Float32
    return Float32(sqrt(v.x*v.x + v.y*v.y + v.z*v.z))
end

# -----------------------------
# Link parameters
# -----------------------------

struct LinkParams
    length_m::Float32
    mass_kg::Float32
    inertia_cm::Float32
    inertia_joint::Float32
end

"""
Create link parameters for a uniform rod model.

- inertia about center of mass (axis ⟂ to rod): I_cm = (1/12) m L^2
- inertia about joint at one end:              I_joint = (1/3)  m L^2
"""
function LinkParams(; length_m=2.5, mass_kg=1.0)
    L = Float32(length_m)
    m = Float32(mass_kg)
    icm = Float32((1/12) * m * L * L)
    ij  = Float32((1/3)  * m * L * L)
    return LinkParams(L, m, icm, ij)
end

# -----------------------------
# Joint / kinematics structs
# -----------------------------

struct JointAngles
    q0_yaw::Float32   # rad
    q1_pitch::Float32 # rad
    q2_pitch::Float32 # rad
end

JointAngles() = JointAngles(0.0f0, 0.0f0, 0.0f0)

struct FKResult
    base::Vec3
    joint1::Vec3
    joint2::Vec3
    ee::Vec3
end

struct IKResult
    reachable::Bool
    q::JointAngles
    message::String
end

# -----------------------------
# Robot arm model (no recursion)
# -----------------------------

struct RobotArm
    link1::LinkParams
    link2::LinkParams
end

# Reach limits
min_reach(arm::RobotArm) = abs(arm.link1.length_m - arm.link2.length_m)
max_reach(arm::RobotArm) = arm.link1.length_m + arm.link2.length_m

# -----------------------------
# Analytic inverse kinematics
# -----------------------------

"""
Analytic IK for a yaw + planar 2-link arm.

Rules enforced:
- target.z >= 0
- ||p|| within [min_reach, max_reach]
"""
function solve_ik(arm::RobotArm, target::Vec3; elbow_up::Bool=false)::IKResult
    # z constraint
    if target.z < 0.0f0
        return IKResult(false, JointAngles(), "Invalid target: z must be >= 0")
    end

    x = Float64(target.x)
    y = Float64(target.y)
    z = Float64(target.z)

    d = sqrt(x*x + y*y + z*z)
    rmin = Float64(min_reach(arm))
    rmax = Float64(max_reach(arm))

    if d < rmin - 1e-9 || d > rmax + 1e-9
        msg = @sprintf("Target radius |p|=%.6f is outside [%.6f, %.6f]", d, rmin, rmax)
        return IKResult(false, JointAngles(), msg)
    end

    # Base yaw
    q0 = (abs(x) > 1e-12 || abs(y) > 1e-12) ? atan(y, x) : 0.0

    # Reduce to planar (r, z)
    r = sqrt(x*x + y*y)
    L1 = Float64(arm.link1.length_m)
    L2 = Float64(arm.link2.length_m)

    # Elbow from law of cosines
    c2 = (r*r + z*z - L1*L1 - L2*L2) / (2.0 * L1 * L2)
    c2 = clamp(c2, -1.0, 1.0)

    q2 = acos(c2)
    if !elbow_up
        q2 = -q2
    end

    s2 = sin(q2)
    k1 = L1 + L2*cos(q2)
    k2 = L2*s2

    q1 = atan(z, r) - atan(k2, k1)

    q = JointAngles(Float32(q0), Float32(q1), Float32(q2))
    return IKResult(true, q, "OK")
end

# -----------------------------
# Forward kinematics
# -----------------------------

function forward_kinematics(arm::RobotArm, q::JointAngles)::FKResult
    base = v3(0,0,0)
    joint1 = base

    L1 = arm.link1.length_m
    L2 = arm.link2.length_m

    cy = Float32(cos(q.q0_yaw))
    sy = Float32(sin(q.q0_yaw))

    # radial direction in XY and +Z axis
    u = v3(cy, sy, 0)
    k = v3(0, 0, 1)

    # elbow
    p1 = (L1 * cos(q.q1_pitch)) * u + (L1 * sin(q.q1_pitch)) * k

    # end effector
    a = q.q1_pitch + q.q2_pitch
    p2 = p1 + (L2 * cos(a)) * u + (L2 * sin(a)) * k

    return FKResult(base, joint1, p1, p2)
end

end # module Robot
