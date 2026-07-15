package manipulator3d.ui

import com.raylib.Raylib
import com.raylib.Colors
import manipulator3d.math.Vec3
import manipulator3d.render.DrawUtils
import manipulator3d.robot.RobotArm

import java.text.{DecimalFormat, DecimalFormatSymbols}
import java.util.Locale

object Overlay {

  final case class OverlayStatus(
    phaseText: String = "",
    errorText: Option[String] = None
  )

  sealed trait FieldId
  object FieldId {
    case object StartX extends FieldId
    case object StartY extends FieldId
    case object StartZ extends FieldId
    case object GoalX extends FieldId
    case object GoalY extends FieldId
    case object GoalZ extends FieldId
  }

  final case class OverlayModel(
    startX: String,
    startY: String,
    startZ: String,
    goalX: String,
    goalY: String,
    goalZ: String,
    focused: Option[FieldId],
    banner: Option[String]
  )

  object OverlayModel {
    private val df: DecimalFormat = {
      val sym = new DecimalFormatSymbols(Locale.US)
      val d = new DecimalFormat("0.###", sym)
      d.setGroupingUsed(false)
      d
    }

    private def fmt(v: Float): String = df.format(v.toDouble)

    def fromTargets(start: Vec3, goal: Vec3): OverlayModel =
      OverlayModel(
        startX = fmt(start.x),
        startY = fmt(start.y),
        startZ = fmt(start.z),
        goalX = fmt(goal.x),
        goalY = fmt(goal.y),
        goalZ = fmt(goal.z),
        focused = None,
        banner = None
      )
  }

  final case class RunRequest(start: Vec3, goal: Vec3, reset: Boolean)

  final case class OverlayOutput(
    paused: Boolean,
    model: OverlayModel,
    runRequest: Option[RunRequest]
  )

  private def pointInRect(p: Raylib.Vector2, r: Raylib.Rectangle): Boolean =
    p.x() >= r.x() && p.x() <= (r.x() + r.width()) &&
      p.y() >= r.y() && p.y() <= (r.y() + r.height())

  private def rect(x: Float, y: Float, w: Float, h: Float): Raylib.Rectangle =
    new Raylib.Rectangle().x(x).y(y).width(w).height(h)

  // jaylib variant doesn't expose thickness, so we fake it by stacking outlines.
  private def drawRoundedOutlineThick(r0: Raylib.Rectangle, roundness: Float, segments: Int, thicknessPx: Int, color: Raylib.Color): Unit = {
    val t = if (thicknessPx < 1) 1 else thicknessPx
    var i = 0
    while (i < t) {
      val x = r0.x() + i
      val y = r0.y() + i
      val w = r0.width() - 2 * i
      val h = r0.height() - 2 * i
      if (w > 0 && h > 0) {
        val r = rect(x, y, w, h)
        Raylib.DrawRectangleRoundedLines(r, roundness, segments, color)
      }
      i += 1
    }
  }

  private def parseFloat(s: String): Option[Float] = {
    val t = s.trim
    if (t.isEmpty) None
    else {
      try Some(t.toFloat)
      catch { case _: Throwable => None }
    }
  }

  private def parseTargets(m: OverlayModel): Either[String, (Vec3, Vec3)] = {
    val sx = parseFloat(m.startX).toRight("START x is not a number")
    val sy = parseFloat(m.startY).toRight("START y is not a number")
    val sz = parseFloat(m.startZ).toRight("START z is not a number")
    val gx = parseFloat(m.goalX).toRight("GOAL x is not a number")
    val gy = parseFloat(m.goalY).toRight("GOAL y is not a number")
    val gz = parseFloat(m.goalZ).toRight("GOAL z is not a number")

    for {
      x1 <- sx
      y1 <- sy
      z1 <- sz
      x2 <- gx
      y2 <- gy
      z2 <- gz
    } yield (Vec3(x1, y1, z1), Vec3(x2, y2, z2))
  }

  private def setField(m: OverlayModel, id: FieldId, value: String): OverlayModel =
    id match {
      case FieldId.StartX => m.copy(startX = value)
      case FieldId.StartY => m.copy(startY = value)
      case FieldId.StartZ => m.copy(startZ = value)
      case FieldId.GoalX  => m.copy(goalX = value)
      case FieldId.GoalY  => m.copy(goalY = value)
      case FieldId.GoalZ  => m.copy(goalZ = value)
    }

  private def getField(m: OverlayModel, id: FieldId): String =
    id match {
      case FieldId.StartX => m.startX
      case FieldId.StartY => m.startY
      case FieldId.StartZ => m.startZ
      case FieldId.GoalX  => m.goalX
      case FieldId.GoalY  => m.goalY
      case FieldId.GoalZ  => m.goalZ
    }

  private def isAllowedChar(ch: Char): Boolean =
    (ch >= '0' && ch <= '9') || ch == '-' || ch == '+' || ch == '.' || ch == 'e' || ch == 'E'

  private def handleTextInput(m0: OverlayModel, paused: Boolean): OverlayModel = {
    if (!paused) return m0

    m0.focused match {
      case None => m0
      case Some(fid) =>
        var m = m0

        if (Raylib.IsKeyPressed(Raylib.KEY_BACKSPACE)) {
          val s = getField(m, fid)
          if (s.nonEmpty) m = setField(m, fid, s.dropRight(1))
        }

        var c = Raylib.GetCharPressed()
        while (c > 0) {
          val ch = c.toChar
          if (isAllowedChar(ch)) {
            val cur = getField(m, fid)
            if (cur.length < 16) m = setField(m, fid, cur + ch)
          }
          c = Raylib.GetCharPressed()
        }

        if (Raylib.IsKeyPressed(Raylib.KEY_DELETE)) {
          m = setField(m, fid, "")
        }

        m
    }
  }

  def drawOverlayPanel(
    font: Raylib.Font,
    arm: RobotArm,
    appliedStart: Vec3,
    appliedGoal: Vec3,
    status: OverlayStatus,
    paused0: Boolean,
    model0: OverlayModel,
    screenW: Int,
    screenH: Int
  ): OverlayOutput = {

    val pad = 12
    val x0 = 14
    val y0 = 14

    // Size changes requested:
    // +30px width, +100px height
    val w = 390
    val h = 620

    Raylib.DrawRectangle(x0, y0, w, h, DrawUtils.color(18, 18, 18, 230))
    Raylib.DrawRectangleLines(x0, y0, w, h, DrawUtils.color(200, 200, 200, 255))

    var y = y0 + pad
    DrawUtils.drawTextBold(font, "3-DOF 2-Link Arm", x0 + pad, y, 24, Colors.RAYWHITE)
    y += 30

    if (status.phaseText.nonEmpty) {
      DrawUtils.drawTextSmall(font, status.phaseText, x0 + pad, y, 18, Colors.SKYBLUE)
      y += 24
    }

    var model = model0.copy(banner = model0.banner.orElse(status.errorText))
    var paused = paused0
    var runReq: Option[RunRequest] = None

    // Pause/Play button
    val btn = rect((x0 + pad).toFloat, y.toFloat, (w - 2 * pad).toFloat, 36.0f)
    val btnBg = if (paused0) DrawUtils.color(60, 120, 60, 220) else DrawUtils.color(120, 60, 60, 220)
    Raylib.DrawRectangleRounded(btn, 0.18f, 8, btnBg)
    drawRoundedOutlineThick(btn, 0.18f, 8, thicknessPx = 2, DrawUtils.color(230, 230, 230, 255))

    val label = if (paused0) "PLAY" else "PAUSE"
    DrawUtils.drawTextBold(font, label, (btn.x() + 10).toInt, (btn.y() + 7).toInt, 22, Colors.RAYWHITE)

    val mp = Raylib.GetMousePosition()

    if (Raylib.IsMouseButtonPressed(Raylib.MOUSE_BUTTON_LEFT) && pointInRect(mp, btn)) {
      if (!paused0) {
        paused = true
        model = model.copy(banner = None)
      } else {
        parseTargets(model) match {
          case Left(err) =>
            paused = true
            model = model.copy(banner = Some(err))

          case Right((s, g)) =>
            if (s.z < 0.0f || g.z < 0.0f) {
              paused = true
              model = model.copy(banner = Some("Invalid input: z must be >= 0"))
            } else {
              val ikS = arm.solveIK(s, elbowUp = false)
              val ikG = arm.solveIK(g, elbowUp = false)

              if (!ikS.reachable) {
                paused = true
                model = model.copy(banner = Some(s"START not reachable: ${ikS.message}"))
              } else if (!ikG.reachable) {
                paused = true
                model = model.copy(banner = Some(s"GOAL not reachable: ${ikG.message}"))
              } else {
                paused = false
                model = model.copy(banner = None)
                val reset = (s != appliedStart) || (g != appliedGoal)
                runReq = Some(RunRequest(s, g, reset))
              }
            }
        }
      }
    }

    y += 48

    DrawUtils.drawTextBold(font, "Targets (meters)", x0 + pad, y, 20, Colors.RAYWHITE)
    y += 26
    DrawUtils.drawTextSmall(font, "Edit only while paused. Click a box to type.", x0 + pad, y, 16, DrawUtils.color(200, 200, 200, 220))
    y += 22

    val boxW = 92.0f
    val boxH = 28.0f
    val gap = 8.0f

    def drawField(lbl: String, fid: FieldId, x: Float, y: Float, enabled: Boolean): Unit = {
      DrawUtils.drawTextSmall(font, lbl, x.toInt, (y - 18).toInt, 16, DrawUtils.color(210, 210, 210, 230))

      val r = rect(x, y, boxW, boxH)
      val focused = model.focused.contains(fid)
      val border = if (focused) DrawUtils.color(255, 255, 255, 255) else DrawUtils.color(150, 150, 150, 255)
      val fill = if (enabled) DrawUtils.color(35, 35, 38, 255) else DrawUtils.color(28, 28, 30, 255)

      Raylib.DrawRectangleRounded(r, 0.15f, 6, fill)
      drawRoundedOutlineThick(r, 0.15f, 6, thicknessPx = 2, border)

      val txt = getField(model, fid)
      DrawUtils.drawTextSmall(font, if (txt.nonEmpty) txt else " ", (x + 6).toInt, (y + 6).toInt, 18, Colors.RAYWHITE)

      if (enabled && Raylib.IsMouseButtonPressed(Raylib.MOUSE_BUTTON_LEFT) && pointInRect(mp, r)) {
        model = model.copy(focused = Some(fid))
      }
    }
    y += 10

    DrawUtils.drawTextSmall(font, "START:", x0 + pad, y + 6, 18, Colors.RAYWHITE)
    val row1x = (x0 + pad + 70).toFloat
    val row1y = y.toFloat
    drawField("x", FieldId.StartX, row1x, row1y, paused)
    drawField("y", FieldId.StartY, row1x + boxW + gap, row1y, paused)
    drawField("z", FieldId.StartZ, row1x + 2 * (boxW + gap), row1y, paused)
    y += 52

    DrawUtils.drawTextSmall(font, "GOAL :", x0 + pad, y + 6, 18, Colors.RAYWHITE)
    val row2x = row1x
    val row2y = y.toFloat
    drawField("x", FieldId.GoalX, row2x, row2y, paused)
    drawField("y", FieldId.GoalY, row2x + boxW + gap, row2y, paused)
    drawField("z", FieldId.GoalZ, row2x + 2 * (boxW + gap), row2y, paused)
    y += 54

    // Drop focus if clicking inside panel but not on a field/button
    if (paused && Raylib.IsMouseButtonPressed(Raylib.MOUSE_BUTTON_LEFT)) {
      val panel = rect(x0.toFloat, y0.toFloat, w.toFloat, h.toFloat)
      val buttonHit = pointInRect(mp, btn)

      val anyFieldHit =
        pointInRect(mp, rect(row1x, row1y, boxW, boxH)) ||
          pointInRect(mp, rect(row1x + boxW + gap, row1y, boxW, boxH)) ||
          pointInRect(mp, rect(row1x + 2 * (boxW + gap), row1y, boxW, boxH)) ||
          pointInRect(mp, rect(row2x, row2y, boxW, boxH)) ||
          pointInRect(mp, rect(row2x + boxW + gap, row2y, boxW, boxH)) ||
          pointInRect(mp, rect(row2x + 2 * (boxW + gap), row2y, boxW, boxH))

      if (pointInRect(mp, panel) && !anyFieldHit && !buttonHit) {
        model = model.copy(focused = None)
      }
    }

    model = handleTextInput(model, paused)

    // Reachability feedback
    val (startOk, goalOk, parseWarn) = parseTargets(model).toOption match {
      case None => (false, false, Some("Invalid numeric format"))
      case Some((s, g)) =>
        if (s.z < 0.0f || g.z < 0.0f) (false, false, Some("z must be >= 0"))
        else {
          val ikS = arm.solveIK(s, elbowUp = false)
          val ikG = arm.solveIK(g, elbowUp = false)
          (ikS.reachable, ikG.reachable, None)
        }
    }

    def norm3(v: Vec3): Float = v.norm

    y += 4
    DrawUtils.drawTextSmall(
      font,
      f"Workspace |p|: [${arm.minReach}%.2f, ${arm.maxReach}%.2f] m",
      x0 + pad, y, 18,
      DrawUtils.color(140, 200, 255, 255)
    )
    y += 26

    val currentStart = parseTargets(model).toOption.map(_._1).getOrElse(appliedStart)
    val currentGoal  = parseTargets(model).toOption.map(_._2).getOrElse(appliedGoal)

    DrawUtils.drawTextSmall(font, f"Start |p|=${norm3(currentStart)}%.3f", x0 + pad, y, 18, if (startOk) Colors.GREEN else Colors.ORANGE); y += 20
    DrawUtils.drawTextSmall(font, f"Start: (${currentStart.x}%.2f, ${currentStart.y}%.2f, ${currentStart.z}%.2f)", x0 + pad, y, 18, if (startOk) Colors.GREEN else Colors.ORANGE); y += 24

    DrawUtils.drawTextSmall(font, f"Goal  |p|=${norm3(currentGoal)}%.3f", x0 + pad, y, 18, if (goalOk) Colors.GREEN else Colors.ORANGE); y += 20
    DrawUtils.drawTextSmall(font, f"Goal : (${currentGoal.x}%.2f, ${currentGoal.y}%.2f, ${currentGoal.z}%.2f)", x0 + pad, y, 18, if (goalOk) Colors.GREEN else Colors.ORANGE); y += 22

    if (!startOk || !goalOk) {
      DrawUtils.drawTextBold(font, "OUT OF REACH!", x0 + pad, y, 22, Colors.RED)
      y += 24
      DrawUtils.drawTextSmall(font, "Choose points inside workspace.", x0 + pad, y, 18, Colors.RED)
      y += 22
      parseWarn.foreach { wmsg =>
        DrawUtils.drawTextSmall(font, wmsg, x0 + pad, y, 18, Colors.RED)
        y += 22
      }
    }

    y += 6
    DrawUtils.drawTextBold(font, "Link parameters", x0 + pad, y, 20, Colors.RAYWHITE)
    y += 24

    val L1 = arm.link1Params
    val L2 = arm.link2Params

    DrawUtils.drawTextSmall(font, f"Link1 length: ${L1.length_m}%.3f m", x0 + pad, y, 18, Colors.RAYWHITE); y += 20
    DrawUtils.drawTextSmall(font, f"Link1 mass  : ${L1.mass_kg}%.3f kg", x0 + pad, y, 18, Colors.RAYWHITE); y += 20
    DrawUtils.drawTextSmall(font, f"Link1 inertia (joint): ${L1.inertia_joint}%.5f", x0 + pad, y, 18, Colors.RAYWHITE); y += 22

    DrawUtils.drawTextSmall(font, f"Link2 length: ${L2.length_m}%.3f m", x0 + pad, y, 18, Colors.RAYWHITE); y += 20
    DrawUtils.drawTextSmall(font, f"Link2 mass  : ${L2.mass_kg}%.3f kg", x0 + pad, y, 18, Colors.RAYWHITE); y += 20
    DrawUtils.drawTextSmall(font, f"Link2 inertia (joint): ${L2.inertia_joint}%.5f", x0 + pad, y, 18, Colors.RAYWHITE); y += 20

    model.banner.foreach { msg =>
      val yy = (y0 + h - 44).toInt
      Raylib.DrawRectangle(x0 + 1, yy, w - 2, 40, DrawUtils.color(60, 20, 20, 220))
      DrawUtils.drawTextSmall(font, msg, x0 + pad, yy + 10, 18, Colors.RAYWHITE)
    }

    OverlayOutput(paused = paused, model = model, runRequest = runReq)
  }
}
