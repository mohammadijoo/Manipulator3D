package manipulator3d.robot

import manipulator3d.math.Vec3

/** Physical parameters for a uniform rod approximation. */
final class LinkParams(var length_m: Float = 2.5f, var mass_kg: Float = 1.0f) {
  var inertia_cm: Float = 0.0f
  var inertia_joint: Float = 0.0f

  def recomputeInertia(): Unit = {
    // Uniform rod about center, axis ⟂ to rod:
    // I_cm = (1/12) m L^2
    // About joint at one end:
    // I_joint = (1/3) m L^2
    inertia_cm = (1.0f / 12.0f) * mass_kg * length_m * length_m
    inertia_joint = (1.0f / 3.0f) * mass_kg * length_m * length_m
  }
}

final case class JointAngles(q0_yaw: Float = 0.0f, q1_pitch: Float = 0.0f, q2_pitch: Float = 0.0f)

final case class FKResult(base: Vec3, joint1: Vec3, joint2: Vec3, ee: Vec3)

final case class IKResult(reachable: Boolean, q: JointAngles, message: String)

/**
  * 3-DOF model:
  *  - q0: base yaw about +Z
  *  - q1: shoulder pitch relative to X–Y plane
  *  - q2: elbow pitch in the same vertical plane
  */
final class RobotArm(l1: LinkParams, l2: LinkParams) {
  private val link1: LinkParams = l1
  private val link2: LinkParams = l2
  link1.recomputeInertia()
  link2.recomputeInertia()

  def L1: Float = link1.length_m
  def L2: Float = link2.length_m

  def maxReach: Float = link1.length_m + link2.length_m
  def minReach: Float = scala.math.abs(link1.length_m - link2.length_m).toFloat

  def link1Params: LinkParams = link1
  def link2Params: LinkParams = link2

  private def clampd(v: Double, lo: Double, hi: Double): Double =
    if (v < lo) lo else if (v > hi) hi else v

  /**
    * Analytic IK: reduces 3D (x,y,z) to planar (r,z) after computing yaw.
    * Constraint: z must be >= 0.
    */
  def solveIK(target: Vec3, elbowUp: Boolean = false): IKResult = {
    if (target.z < 0.0f) {
      return IKResult(reachable = false, JointAngles(), "Invalid target: z must be >= 0")
    }

    val x = target.x.toDouble
    val y = target.y.toDouble
    val z = target.z.toDouble

    val d = scala.math.sqrt(x * x + y * y + z * z)
    val rmin = minReach.toDouble
    val rmax = maxReach.toDouble

    if (d < rmin - 1e-9 || d > rmax + 1e-9) {
      return IKResult(
        reachable = false,
        JointAngles(),
        f"Target radius |p|=$d%.4f is outside [$rmin%.4f, $rmax%.4f]"
      )
    }

    // Base yaw in XY
    val q0 = if (scala.math.abs(x) > 1e-12 || scala.math.abs(y) > 1e-12) scala.math.atan2(y, x) else 0.0

    // Planar reduction in (r,z)
    val r = scala.math.sqrt(x * x + y * y)
    val L1d = link1.length_m.toDouble
    val L2d = link2.length_m.toDouble

    // Law of cosines for elbow
    var c2 = (r * r + z * z - L1d * L1d - L2d * L2d) / (2.0 * L1d * L2d)
    c2 = clampd(c2, -1.0, 1.0)

    var q2 = scala.math.acos(c2)
    if (!elbowUp) q2 = -q2

    val s2 = scala.math.sin(q2)
    val k1 = L1d + L2d * scala.math.cos(q2)
    val k2 = L2d * s2

    val q1 = scala.math.atan2(z, r) - scala.math.atan2(k2, k1)

    IKResult(
      reachable = true,
      JointAngles(q0.toFloat, q1.toFloat, q2.toFloat),
      "OK"
    )
  }

  /** FK: returns base, elbow, and end-effector positions for rendering. */
  def forwardKinematics(q: JointAngles): FKResult = {
    val base = Vec3.Zero
    val joint1 = base

    val L1 = link1.length_m
    val L2 = link2.length_m

    val cy = scala.math.cos(q.q0_yaw.toDouble).toFloat
    val sy = scala.math.sin(q.q0_yaw.toDouble).toFloat

    val u = Vec3(cy, sy, 0.0f) // radial direction in XY
    val k = Vec3.UnitZ

    val p1 = (u * (L1 * scala.math.cos(q.q1_pitch.toDouble).toFloat)) +
      (k * (L1 * scala.math.sin(q.q1_pitch.toDouble).toFloat))

    val a = q.q1_pitch + q.q2_pitch
    val p2 = p1 +
      (u * (L2 * scala.math.cos(a.toDouble).toFloat)) +
      (k * (L2 * scala.math.sin(a.toDouble).toFloat))

    FKResult(base = base, joint1 = joint1, joint2 = p1, ee = p2)
  }
}
