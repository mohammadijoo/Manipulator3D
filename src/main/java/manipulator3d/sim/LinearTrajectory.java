package manipulator3d.sim;

import manipulator3d.robot.Vec3f;

/** Linear interpolation trajectory in 3D with a fixed duration. */
public final class LinearTrajectory {
    private final Vec3f a = new Vec3f();
    private final Vec3f b = new Vec3f();
    private float duration = 1.0f;
    private float t = 0.0f;
    private float alpha = 0.0f;
    private boolean finished = true;

    public void reset(Vec3f from, Vec3f to, float durationSec) {
        a.set(from);
        b.set(to);
        duration = (durationSec > 1e-6f) ? durationSec : 1e-6f;
        t = 0.0f;
        alpha = 0.0f;
        finished = false;
    }

    public void update(float dt) {
        if (finished) return;
        t += dt;
        alpha = clamp(t / duration, 0.0f, 1.0f);
        if (alpha >= 1.0f) finished = true;
    }

    public Vec3f position() {
        return new Vec3f(
                a.x + (b.x - a.x) * alpha,
                a.y + (b.y - a.y) * alpha,
                a.z + (b.z - a.z) * alpha
        );
    }

    public boolean finished() { return finished; }
    public float alpha() { return alpha; }

    private static float clamp(float v, float lo, float hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}
