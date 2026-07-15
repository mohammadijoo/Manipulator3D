package manipulator3d.robot;

/** Forward-kinematics positions used for rendering. */
public final class FKResult {
    public final Vec3f base   = new Vec3f(0,0,0);
    public final Vec3f joint1 = new Vec3f(0,0,0); // same as base for this model
    public final Vec3f joint2 = new Vec3f(0,0,0); // elbow
    public final Vec3f ee     = new Vec3f(0,0,0); // end-effector
}
