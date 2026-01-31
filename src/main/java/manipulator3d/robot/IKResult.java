package manipulator3d.robot;

/** Inverse-kinematics result, including a reachability flag and diagnostic message. */
public final class IKResult {
    public boolean reachable = false;
    public final JointAngles q = new JointAngles();
    public String message = "";

    public static IKResult unreachable(String msg) {
        IKResult r = new IKResult();
        r.reachable = false;
        r.message = msg;
        return r;
    }
}
