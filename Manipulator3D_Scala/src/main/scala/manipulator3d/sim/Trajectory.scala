package manipulator3d.sim

import manipulator3d.math.Vec3
import manipulator3d.math.Vec3.clamp

final class LinearTrajectory {
  private var a: Vec3 = Vec3.Zero
  private var b: Vec3 = Vec3.Zero
  private var duration: Float = 1.0f
  private var t: Float = 0.0f
  private var alpha: Float = 0.0f
  private var finished: Boolean = true

  def reset(from: Vec3, to: Vec3, durationSec: Float): Unit = {
    a = from
    b = to
    duration = if (durationSec > 1e-6f) durationSec else 1e-6f
    t = 0.0f
    alpha = 0.0f
    finished = false
  }

  def update(dt: Float): Unit = {
    if (finished) return
    t += dt
    alpha = clamp(t / duration, 0.0f, 1.0f)
    if (alpha >= 1.0f) finished = true
  }

  def position: Vec3 =
    Vec3(
      a.x + (b.x - a.x) * alpha,
      a.y + (b.y - a.y) * alpha,
      a.z + (b.z - a.z) * alpha
    )

  def isFinished: Boolean = finished
  def getAlpha: Float = alpha
}
