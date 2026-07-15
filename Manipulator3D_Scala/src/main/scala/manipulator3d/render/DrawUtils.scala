package manipulator3d.render

import com.raylib.Raylib
import manipulator3d.math.Vec3

import java.lang.reflect.Method
import java.nio.charset.StandardCharsets
import org.bytedeco.javacpp.BytePointer

object DrawUtils {

  private def v2(x: Float, y: Float): Raylib.Vector2 =
    new Raylib.Vector2().x(x).y(y)

  def vec3(v: Vec3): Raylib.Vector3 =
    new Raylib.Vector3().x(v.x).y(v.y).z(v.z)

  def vec3(x: Float, y: Float, z: Float): Raylib.Vector3 =
    new Raylib.Vector3().x(x).y(y).z(z)

  def color(r: Int, g: Int, b: Int, a: Int): Raylib.Color =
    new Raylib.Color().r(r.toByte).g(g.toByte).b(b.toByte).a(a.toByte)

  // ------------------------------------------------------------
  // Text rendering (portable + correct spacing + NO JNI crashes)
  //
  // We avoid passing Java String directly to JNI.
  // Instead we call DrawTextEx(Font, BytePointer, ...) with a UTF-8 *NUL-terminated* buffer.
  //
  // IMPORTANT: The buffer MUST end with '\0' or raylib will read beyond memory and crash.
  // ------------------------------------------------------------

  private lazy val mDrawTextExBytePtr: Option[Method] = {
    val cls = classOf[Raylib]
    cls.getMethods.find { m =>
      m.getName == "DrawTextEx" &&
      m.getParameterCount == 6 &&
      classOf[BytePointer].isAssignableFrom(m.getParameterTypes.apply(1))
    }
  }

  private def utf8z(s: String): BytePointer = {
    // UTF-8 bytes + explicit NUL terminator
    val raw = s.getBytes(StandardCharsets.UTF_8)
    val z = new Array[Byte](raw.length + 1)
    System.arraycopy(raw, 0, z, 0, raw.length)
    z(raw.length) = 0.toByte

    // Construct BytePointer from the NUL-terminated byte array
    new BytePointer(z: _*)
  }

  private def drawTextExAt(font: Raylib.Font, text: String, x: Int, y: Int, fontSize: Float, c: Raylib.Color): Unit = {
    if (text == null || text.isEmpty) return

    val pos = v2(x.toFloat, y.toFloat)

    // Extra spacing between glyphs (pixels). Keep at 0 to avoid widening.
    val spacing = 0.0f

    mDrawTextExBytePtr match {
      case Some(m) =>
        val bp = utf8z(text)
        try {
          m.invoke(
            null,
            font,
            bp,
            pos,
            java.lang.Float.valueOf(fontSize),
            java.lang.Float.valueOf(spacing),
            c
          )
        } finally {
          // Free native buffer immediately after call (safe because raylib consumes it inside the call)
          try bp.close() catch { case _: Throwable => () }
        }

      case None =>
        // Fallback only if your jaylib build doesn't expose BytePointer overload.
        // If you ever see "?????" again here, it means your environment's String->JNI path is broken.
        Raylib.DrawTextEx(font, text, pos, fontSize, spacing, c)
    }
  }

  def drawTextBold(font: Raylib.Font, text: String, x: Int, y: Int, fontSize: Float, c: Raylib.Color): Unit = {
    drawTextExAt(font, text, x, y, fontSize, c)
    drawTextExAt(font, text, x + 1, y, fontSize, c)
    drawTextExAt(font, text, x, y + 1, fontSize, c)
    drawTextExAt(font, text, x + 1, y + 1, fontSize, c)
  }

  def drawTextSmall(font: Raylib.Font, text: String, x: Int, y: Int, fontSize: Float, c: Raylib.Color): Unit =
    drawTextExAt(font, text, x, y, fontSize, c)

  // ---------------------------
  // Robot visuals
  // ---------------------------

  def drawRobotBasePedestal(origin: Vec3): Unit = {
    val a = vec3(origin.x, origin.y, -0.25f)
    val b = vec3(origin.x, origin.y, 0.00f)
    Raylib.DrawCylinderEx(a, b, 0.55f, 0.55f, 24, color(70, 70, 75, 255))

    val c0 = vec3(origin.x, origin.y, 0.00f)
    val d = vec3(origin.x, origin.y, 0.35f)
    Raylib.DrawCylinderEx(c0, d, 0.38f, 0.34f, 24, color(95, 95, 100, 255))

    Raylib.DrawCylinderEx(
      vec3(origin.x, origin.y, 0.00f),
      vec3(origin.x, origin.y, 0.06f),
      0.48f,
      0.48f,
      24,
      color(110, 110, 115, 255)
    )
  }

  def drawRobotJointHousing(center: Vec3, radius: Float): Unit = {
    val c = vec3(center)
    Raylib.DrawSphere(c, radius, color(120, 120, 125, 255))
    Raylib.DrawSphereWires(c, radius, 12, 12, color(200, 200, 200, 60))

    Raylib.DrawCylinderEx(
      vec3(center.x, center.y, center.z - 0.10f),
      vec3(center.x, center.y, center.z + 0.10f),
      radius * 0.55f,
      radius * 0.55f,
      18,
      color(85, 85, 90, 255)
    )
  }

  def drawTaperedLink(a: Vec3, b: Vec3, rA: Float, rB: Float, col: Raylib.Color): Unit = {
    Raylib.DrawCylinderEx(vec3(a), vec3(b), rA, rB, 20, col)
    Raylib.DrawSphere(vec3(a), rA * 0.95f, color(140, 140, 145, 255))
    Raylib.DrawSphere(vec3(b), rB * 0.95f, color(140, 140, 145, 255))
  }

  def drawSuctionTool(ee: Vec3, approachDir: Vec3): Unit = {
    val tip = ee + (approachDir * 0.28f)
    Raylib.DrawCylinderEx(vec3(ee), vec3(tip), 0.06f, 0.05f, 18, color(40, 40, 45, 255))

    val cupA = tip
    val cupB = tip + (approachDir * 0.06f)
    Raylib.DrawCylinderEx(vec3(cupA), vec3(cupB), 0.11f, 0.11f, 24, color(25, 25, 28, 255))

    Raylib.DrawSphere(vec3(tip), 0.035f, color(80, 80, 85, 255))
  }
}
