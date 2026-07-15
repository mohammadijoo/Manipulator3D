using System;
using System.Numerics;

namespace Manipulator3D.robot;

public struct LinkParams
{
    public float length_m;
    public float mass_kg;

    // Uniform rod approximations (about center, axis perpendicular to rod).
    // I_cm   = (1/12) m L^2
    // I_joint= (1/3)  m L^2
    public float inertia_cm;
    public float inertia_joint;

    public void RecomputeInertia()
    {
        inertia_cm = (1.0f / 12.0f) * mass_kg * length_m * length_m;
        inertia_joint = (1.0f / 3.0f) * mass_kg * length_m * length_m;
    }
}

public struct JointAngles
{
    public float q0_yaw;   // rad
    public float q1_pitch; // rad (relative to XY plane)
    public float q2_pitch; // rad (elbow bend in same plane)
}

public struct FKResult
{
    public Vector3 basePos;
    public Vector3 joint1; // shoulder location (same as base here)
    public Vector3 joint2; // elbow
    public Vector3 ee;     // end effector
}

public struct IKResult
{
    public bool reachable;
    public JointAngles q;
    public string message;
}

public sealed class RobotArm
{
    private LinkParams _link1;
    private LinkParams _link2;

    public RobotArm(LinkParams l1, LinkParams l2)
    {
        _link1 = l1;
        _link2 = l2;
        _link1.RecomputeInertia();
        _link2.RecomputeInertia();
    }

    public float MaxReach() => _link1.length_m + _link2.length_m;
    public float MinReach() => MathF.Abs(_link1.length_m - _link2.length_m);

    public ref readonly LinkParams Link1() => ref _link1;
    public ref readonly LinkParams Link2() => ref _link2;

    public IKResult SolveIK(Vector3 target, bool elbowUp = false)
    {
        IKResult outp = new() { reachable = false, message = "" };

        // Workspace rule for this simulation: z must be non-negative.
        if (target.Z < 0.0f)
        {
            outp.message = "Invalid target: z must be >= 0";
            return outp;
        }

        double x = target.X;
        double y = target.Y;
        double z = target.Z;

        double d = Math.Sqrt(x * x + y * y + z * z);
        double rmin = MinReach();
        double rmax = MaxReach();

        if (d < rmin - 1e-9 || d > rmax + 1e-9)
        {
            outp.message = $"Target radius |p|={d:0.###} is outside [{rmin:0.###}, {rmax:0.###}]";
            return outp;
        }

        // Base yaw in XY plane.
        double q0 = 0.0;
        if (Math.Abs(x) > 1e-12 || Math.Abs(y) > 1e-12)
            q0 = Math.Atan2(y, x);

        // Reduce to planar IK in (r,z), where r = sqrt(x^2 + y^2).
        double r = Math.Sqrt(x * x + y * y);
        double L1 = _link1.length_m;
        double L2 = _link2.length_m;

        // Elbow angle (law of cosines).
        double c2 = (r * r + z * z - L1 * L1 - L2 * L2) / (2.0 * L1 * L2);
        c2 = Clamp(c2, -1.0, 1.0);

        double q2 = Math.Acos(c2);
        if (!elbowUp) q2 = -q2; // default branch (elbow-down)

        // Shoulder angle.
        double s2 = Math.Sin(q2);
        double k1 = L1 + L2 * Math.Cos(q2);
        double k2 = L2 * s2;

        double q1 = Math.Atan2(z, r) - Math.Atan2(k2, k1);

        outp.reachable = true;
        outp.q.q0_yaw = (float)q0;
        outp.q.q1_pitch = (float)q1;
        outp.q.q2_pitch = (float)q2;
        outp.message = "OK";
        return outp;
    }

    public FKResult ForwardKinematics(JointAngles q)
    {
        FKResult fk = new();
        fk.basePos = Vector3.Zero;
        fk.joint1 = fk.basePos;

        float L1 = _link1.length_m;
        float L2 = _link2.length_m;

        // u = radial direction in XY defined by yaw, k = +Z
        float cy = MathF.Cos(q.q0_yaw);
        float sy = MathF.Sin(q.q0_yaw);
        Vector3 u = new(cy, sy, 0.0f);
        Vector3 k = Vector3.UnitZ;

        // Elbow
        Vector3 p1 = u * (L1 * MathF.Cos(q.q1_pitch)) + k * (L1 * MathF.Sin(q.q1_pitch));

        // End effector
        float a = q.q1_pitch + q.q2_pitch;
        Vector3 p2 = p1 + u * (L2 * MathF.Cos(a)) + k * (L2 * MathF.Sin(a));

        fk.joint2 = p1;
        fk.ee = p2;
        return fk;
    }

    private static double Clamp(double v, double lo, double hi)
        => Math.Max(lo, Math.Min(hi, v));
}
