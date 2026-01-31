package manipulator3d.robot;

/** Joint variables for the 3-DOF arm. */
public final class JointAngles {
    public float q0Yaw   = 0.0f; // rad
    public float q1Pitch = 0.0f; // rad (relative to XY plane)
    public float q2Pitch = 0.0f; // rad (elbow relative, same plane)

    public JointAngles() {}
    public JointAngles(float q0Yaw, float q1Pitch, float q2Pitch) {
        this.q0Yaw = q0Yaw;
        this.q1Pitch = q1Pitch;
        this.q2Pitch = q2Pitch;
    }

    public JointAngles set(JointAngles o) {
        this.q0Yaw = o.q0Yaw;
        this.q1Pitch = o.q1Pitch;
        this.q2Pitch = o.q2Pitch;
        return this;
    }
}
