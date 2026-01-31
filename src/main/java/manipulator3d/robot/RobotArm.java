package manipulator3d.robot;

/**
 * 3-DOF two-link arm model:
 * - base yaw about +Z
 * - two pitch joints in a vertical plane defined by yaw
 *
 * Provides closed-form IK and FK.
 */
public final class RobotArm {
    private final LinkParams link1;
    private final LinkParams link2;

    public RobotArm(LinkParams l1, LinkParams l2) {
        this.link1 = l1;
        this.link2 = l2;
        this.link1.recomputeInertia();
        this.link2.recomputeInertia();
    }

    public float L1() { return link1.lengthM; }
    public float L2() { return link2.lengthM; }

    public float maxReach() { return link1.lengthM + link2.lengthM; }
    public float minReach() { return Math.abs(link1.lengthM - link2.lengthM); }

    public LinkParams link1() { return link1; }
    public LinkParams link2() { return link2; }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    /**
     * Analytic IK for a 3D Cartesian target.
     * @param target end-effector target in meters
     * @param elbowUp choose elbow-up vs elbow-down branch
     */
    public IKResult solveIK(Vec3f target, boolean elbowUp) {
        if (target.z < 0.0f) {
            return IKResult.unreachable("Invalid target: z must be >= 0");
        }

        double x = target.x, y = target.y, z = target.z;

        double d = Math.sqrt(x*x + y*y + z*z);
        double rmin = minReach();
        double rmax = maxReach();

        if (d < rmin - 1e-9 || d > rmax + 1e-9) {
            return IKResult.unreachable(String.format(
                    "Target radius |p|=%.6f is outside [%.6f, %.6f]", d, rmin, rmax));
        }

        // Base yaw in XY plane
        double q0 = 0.0;
        if (Math.abs(x) > 1e-12 || Math.abs(y) > 1e-12) {
            q0 = Math.atan2(y, x);
        }

        // Planar reduction in (r, z) where r = sqrt(x^2+y^2)
        double r = Math.sqrt(x*x + y*y);
        double L1 = link1.lengthM;
        double L2 = link2.lengthM;

        // Elbow from law of cosines
        double c2 = (r*r + z*z - L1*L1 - L2*L2) / (2.0 * L1 * L2);
        c2 = clamp(c2, -1.0, 1.0);

        double q2 = Math.acos(c2);
        if (!elbowUp) q2 = -q2; // elbow-down default

        // Shoulder angle relative to XY plane
        double s2 = Math.sin(q2);
        double k1 = L1 + L2 * Math.cos(q2);
        double k2 = L2 * s2;

        double q1 = Math.atan2(z, r) - Math.atan2(k2, k1);

        IKResult out = new IKResult();
        out.reachable = true;
        out.q.q0Yaw = (float) q0;
        out.q.q1Pitch = (float) q1;
        out.q.q2Pitch = (float) q2;
        out.message = "OK";
        return out;
    }

    /** Forward kinematics: joint2 (elbow) and ee positions. */
    public FKResult forwardKinematics(JointAngles q) {
        FKResult fk = new FKResult();

        float L1 = link1.lengthM;
        float L2 = link2.lengthM;

        float cy = (float)Math.cos(q.q0Yaw);
        float sy = (float)Math.sin(q.q0Yaw);

        // Radial direction in XY plane and +Z axis
        Vec3f u = new Vec3f(cy, sy, 0.0f);
        Vec3f k = new Vec3f(0.0f, 0.0f, 1.0f);

        // Elbow
        Vec3f p1 = Vec3f.add(
                Vec3f.scale(u, L1 * (float)Math.cos(q.q1Pitch)),
                Vec3f.scale(k, L1 * (float)Math.sin(q.q1Pitch))
        );

        // End-effector
        float a = q.q1Pitch + q.q2Pitch;
        Vec3f p2 = Vec3f.add(
                p1,
                Vec3f.add(
                        Vec3f.scale(u, L2 * (float)Math.cos(a)),
                        Vec3f.scale(k, L2 * (float)Math.sin(a))
                )
        );

        fk.joint2.set(p1);
        fk.ee.set(p2);
        return fk;
    }
}
