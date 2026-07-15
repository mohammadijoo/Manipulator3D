package manipulator3d.math

/** Small, allocation-free math type for the simulation update loop. */
final case class Vec3(x: Float, y: Float, z: Float) {
  def +(o: Vec3): Vec3 = Vec3(x + o.x, y + o.y, z + o.z)
  def -(o: Vec3): Vec3 = Vec3(x - o.x, y - o.y, z - o.z)
  def *(s: Float): Vec3 = Vec3(x * s, y * s, z * s)

  def dot(o: Vec3): Float = x * o.x + y * o.y + z * o.z
  def norm: Float = math.sqrt(dot(this)).toFloat

  def normalized: Vec3 = {
    val n = norm
    if (n <= 1e-9f) Vec3(0f, 0f, 0f) else this * (1.0f / n)
  }
}

object Vec3 {
  val Zero: Vec3 = Vec3(0f, 0f, 0f)
  val UnitZ: Vec3 = Vec3(0f, 0f, 1f)

  def clamp(v: Float, lo: Float, hi: Float): Float =
    if (v < lo) lo else if (v > hi) hi else v
}
