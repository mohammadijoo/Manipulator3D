package manipulator3d.robot;

/**
 * Minimal float vector used for kinematics and trajectory math.
 * Kept pure-Java to avoid frequent native memory accesses in the update loop.
 */
public final class Vec3f {
    public float x, y, z;

    public Vec3f() { this(0, 0, 0); }
    public Vec3f(float x, float y, float z) { this.x = x; this.y = y; this.z = z; }

    public Vec3f set(float x, float y, float z) { this.x = x; this.y = y; this.z = z; return this; }
    public Vec3f set(Vec3f o) { return set(o.x, o.y, o.z); }

    public static Vec3f add(Vec3f a, Vec3f b) { return new Vec3f(a.x + b.x, a.y + b.y, a.z + b.z); }
    public static Vec3f sub(Vec3f a, Vec3f b) { return new Vec3f(a.x - b.x, a.y - b.y, a.z - b.z); }
    public static Vec3f scale(Vec3f v, float s) { return new Vec3f(v.x * s, v.y * s, v.z * s); }

    public static float dot(Vec3f a, Vec3f b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
    public static float norm(Vec3f v) { return (float)Math.sqrt(dot(v, v)); }

    public static Vec3f normalize(Vec3f v) {
        float n = norm(v);
        if (n < 1e-9f) return new Vec3f(1, 0, 0);
        return scale(v, 1.0f / n);
    }
}
