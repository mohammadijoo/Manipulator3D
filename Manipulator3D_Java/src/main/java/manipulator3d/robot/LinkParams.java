package manipulator3d.robot;

/**
 * Physical and geometric parameters for a single rigid link.
 * Inertia terms assume a uniform rod model.
 */
public final class LinkParams {
    public float lengthM = 2.5f;
    public float massKg  = 1.0f;

    // Uniform rod approximations (axis perpendicular to rod).
    // I_cm   = (1/12) m L^2  (about center)
    // I_joint= (1/3)  m L^2  (about one end)
    public float inertiaCm    = 0.0f;
    public float inertiaJoint = 0.0f;

    public void recomputeInertia() {
        inertiaCm = (1.0f / 12.0f) * massKg * lengthM * lengthM;
        inertiaJoint = (1.0f / 3.0f) * massKg * lengthM * lengthM;
    }
}
