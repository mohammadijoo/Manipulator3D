package manipulator3d

import com.raylib.Raylib
import com.raylib.Colors
import manipulator3d.math.Vec3
import manipulator3d.render.DrawUtils
import manipulator3d.robot.{JointAngles, LinkParams, RobotArm}
import manipulator3d.sim.LinearTrajectory
import manipulator3d.ui.Overlay

import java.io.File
import java.nio.file.{Files, StandardCopyOption}
import java.util.Locale

object Main {

  private sealed trait Phase
  private object Phase {
    case object Idle extends Phase
    case object MoveHomeToStart extends Phase
    case object PickAtStart extends Phase
    case object MoveStartToGoal extends Phase
    case object PlaceAtGoal extends Phase
    case object ReturnGoalToHome extends Phase
    case object WaitAtHomeReset extends Phase
    case object Error extends Phase
  }

  private sealed trait BallState
  private object BallState {
    case object AtStart extends BallState
    case object Attached extends BallState
    case object AtGoal extends BallState
  }

  // UI codepoints: ASCII printable range (fast + sufficient for your UI text)
  // If you later add non-ASCII symbols, extend this range or add specific codepoints.
  private val UiCodepoints: Array[Int] = (32 to 126).toArray

  private def fontOk(f: Raylib.Font): Boolean =
    try f.texture().id() != 0 catch { case _: Throwable => true }

  private def loadBundledInterFont(px: Int): Option[Raylib.Font] = {
    // Put Inter here (portable, no machine-specific path):
    //   src/main/resources/fonts/Inter-Regular.ttf
    val resourcePath = "fonts/Inter-Regular.ttf"
    val cl = this.getClass.getClassLoader
    val is = cl.getResourceAsStream(resourcePath)
    if (is == null) return None

    val tmp = File.createTempFile("inter_regular_", ".ttf")
    tmp.deleteOnExit()

    try {
      Files.copy(is, tmp.toPath, StandardCopyOption.REPLACE_EXISTING)
    } finally {
      try is.close() catch { case _: Throwable => () }
    }

    val f = Raylib.LoadFontEx(tmp.getAbsolutePath, px, UiCodepoints, UiCodepoints.length)
    if (!fontOk(f)) None
    else {
      try Raylib.SetTextureFilter(f.texture(), Raylib.TEXTURE_FILTER_BILINEAR)
      catch { case _: Throwable => () }
      Some(f)
    }
  }

  private def loadFontFallbackFromRelativePaths(px: Int): Option[Raylib.Font] = {
    // Still portable (project-relative), but not required if you use src/main/resources.
    val candidates = Seq(
      "src/main/resources/fonts/Inter-Regular.ttf",
      "resources/fonts/Inter-Regular.ttf",
      "resource/fonts/Inter-Regular.ttf",
      "../src/main/resources/fonts/Inter-Regular.ttf",
      "../resources/fonts/Inter-Regular.ttf",
      "../resource/fonts/Inter-Regular.ttf"
    )

    candidates.iterator
      .filter(Raylib.FileExists)
      .map { path =>
        val f = Raylib.LoadFontEx(path, px, UiCodepoints, UiCodepoints.length)
        if (fontOk(f)) {
          try Raylib.SetTextureFilter(f.texture(), Raylib.TEXTURE_FILTER_BILINEAR)
          catch { case _: Throwable => () }
          Some(f)
        } else None
      }
      .collectFirst { case Some(f) => f }
  }

  private def loadBestUIFont(px: Int): Raylib.Font = {
    loadBundledInterFont(px)
      .orElse(loadFontFallbackFromRelativePaths(px))
      .getOrElse {
        val defFont = Raylib.GetFontDefault()
        try Raylib.SetTextureFilter(defFont.texture(), Raylib.TEXTURE_FILTER_BILINEAR)
        catch { case _: Throwable => () }
        defFont
      }
  }

  private def updateZoom(cam: Raylib.Camera3D): Unit = {
    val wheel = Raylib.GetMouseWheelMove()
    if (wheel == 0.0f) return

    val pos = cam._position()
    val tgt = cam.target()

    val v = Vec3(pos.x(), pos.y(), pos.z()) - Vec3(tgt.x(), tgt.y(), tgt.z())
    var dist = v.norm
    if (dist < 0.001f) dist = 0.001f

    var scale = 1.0f - wheel * 0.10f
    scale = Vec3.clamp(scale, 0.70f, 1.30f)

    dist *= scale
    dist = Vec3.clamp(dist, 1.0f, 200.0f)

    val dir = v.normalized
    val newPos = Vec3(tgt.x(), tgt.y(), tgt.z()) + (dir * dist)
    cam._position(DrawUtils.vec3(newPos))
  }

  private def approxEq(a: Vec3, b: Vec3, eps: Float = 1e-6f): Boolean =
    (scala.math.abs(a.x - b.x) <= eps) &&
      (scala.math.abs(a.y - b.y) <= eps) &&
      (scala.math.abs(a.z - b.z) <= eps)

  def main(args: Array[String]): Unit = {
    // Keep formatting consistent regardless of OS language
    Locale.setDefault(Locale.US)

    Raylib.SetTraceLogLevel(Raylib.LOG_ERROR)
    Raylib.SetConfigFlags(Raylib.FLAG_WINDOW_RESIZABLE | Raylib.FLAG_MSAA_4X_HINT)

    // ---- Arm parameters ----
    val link1 = new LinkParams(length_m = 3.0f, mass_kg = 2.0f); link1.recomputeInertia()
    val link2 = new LinkParams(length_m = 2.6f, mass_kg = 1.6f); link2.recomputeInertia()
    val arm = new RobotArm(link1, link2)

    // Defaults requested by you
    var start = Vec3(1.0f, 2.0f, 1.0f)
    var goal  = Vec3(3.0f, 2.0f, 4.0f)

    var appliedStart = start
    var appliedGoal  = goal

    val homeEE = Vec3(2.0f, 2.0f, 2.0f)
    val ikHome = arm.solveIK(homeEE, elbowUp = false)

    Raylib.InitWindow(1280, 720, "Manipulator3D - 3DOF IK Pick&Place (Scala)")
    Raylib.SetWindowMinSize(960, 540)
    Raylib.SetTargetFPS(60)

    // Load Inter from classpath (portable) or fallback
    val uiFontSizePx = 22
    val uiFont = loadBestUIFont(uiFontSizePx)

    // Camera framing
    val reach = arm.maxReach
    val cam = new Raylib.Camera3D()
      .target(DrawUtils.vec3(0.0f, 0.0f, 0.35f * reach))
      .up(DrawUtils.vec3(0.0f, 0.0f, 1.0f))
      .fovy(52.0f)
      .projection(Raylib.CAMERA_PERSPECTIVE)
      ._position(DrawUtils.vec3(1.10f * reach, -1.15f * reach, 0.85f * reach))

    var overlayModel = Overlay.OverlayModel.fromTargets(start, goal)

    // Timing
    val moveHomeToStart = 2.2f
    val pickDuration = 0.45f
    val moveStartToGoal = 2.6f
    val placeDuration = 0.35f
    val returnToHome = 2.0f
    val resetWaitTotal = 1.5f

    var timer = 0.0f

    val ballRadius = Vec3.clamp(0.03f * reach, 0.06f, 0.16f)

    var ballState: BallState = BallState.AtStart
    var ballPos: Vec3 = start

    var targetEE: Vec3 = homeEE
    var qcmd: JointAngles = if (ikHome.reachable) ikHome.q else JointAngles()
    var runtimeError: Option[String] = None

    val traj = new LinearTrajectory()
    var phase: Phase = Phase.Idle

    // Start paused; user presses PLAY
    var paused = true

    def resetSimulation(newStart: Vec3, newGoal: Vec3): Unit = {
      start = newStart
      goal = newGoal
      appliedStart = newStart
      appliedGoal = newGoal
      runtimeError = None

      val ikS = arm.solveIK(start, elbowUp = false)
      val ikG = arm.solveIK(goal, elbowUp = false)

      if (!ikHome.reachable) {
        phase = Phase.Error
        runtimeError = Some(s"HOME not reachable: ${ikHome.message}")
      } else if (!ikS.reachable) {
        phase = Phase.Error
        runtimeError = Some(s"START not reachable: ${ikS.message}")
      } else if (!ikG.reachable) {
        phase = Phase.Error
        runtimeError = Some(s"GOAL not reachable: ${ikG.message}")
      } else {
        qcmd = ikHome.q
        targetEE = homeEE

        ballState = BallState.AtStart
        ballPos = start

        traj.reset(homeEE, start, moveHomeToStart)
        phase = Phase.MoveHomeToStart
        timer = 0.0f
      }
    }

    val bg = DrawUtils.color(10, 12, 16, 255)
    val uiHint = "F11: fullscreen   Mouse Wheel: zoom   (Edit START/GOAL in panel while paused)"

    while (!Raylib.WindowShouldClose()) {
      if (Raylib.IsKeyPressed(Raylib.KEY_F11)) Raylib.ToggleFullscreen()

      val screenW = Raylib.GetScreenWidth()
      val screenH = Raylib.GetScreenHeight()

      updateZoom(cam)

      val dt = if (paused) 0.0f else Raylib.GetFrameTime()
      runtimeError = None

      if (phase != Phase.Error && phase != Phase.Idle) {
        phase match {
          case Phase.MoveHomeToStart =>
            ballState = BallState.AtStart
            ballPos = start

            traj.update(dt)
            targetEE = traj.position

            if (traj.isFinished) {
              phase = Phase.PickAtStart
              timer = 0.0f
              targetEE = start
            }

          case Phase.PickAtStart =>
            targetEE = start
            ballState = BallState.AtStart
            ballPos = start

            timer += dt
            if (timer >= pickDuration) {
              ballState = BallState.Attached
              timer = 0.0f
              traj.reset(start, goal, moveStartToGoal)
              phase = Phase.MoveStartToGoal
            }

          case Phase.MoveStartToGoal =>
            traj.update(dt)
            targetEE = traj.position
            ballState = BallState.Attached

            if (traj.isFinished) {
              phase = Phase.PlaceAtGoal
              timer = 0.0f
              targetEE = goal
            }

          case Phase.PlaceAtGoal =>
            targetEE = goal
            timer += dt

            if (timer >= placeDuration) {
              ballState = BallState.AtGoal
              timer = 0.0f
              traj.reset(goal, homeEE, returnToHome)
              phase = Phase.ReturnGoalToHome
            } else {
              ballState = BallState.Attached
            }

          case Phase.ReturnGoalToHome =>
            ballState = BallState.AtGoal
            traj.update(dt)
            targetEE = traj.position
            timer += dt

            if (traj.isFinished) {
              phase = Phase.WaitAtHomeReset
            }

          case Phase.WaitAtHomeReset =>
            targetEE = homeEE
            timer += dt

            if (timer >= resetWaitTotal) {
              ballState = BallState.AtStart
              ballPos = start
              timer = 0.0f
              traj.reset(homeEE, start, moveHomeToStart)
              phase = Phase.MoveHomeToStart
            } else {
              ballState = BallState.AtGoal
            }

          case _ => ()
        }

        val ikNow = arm.solveIK(targetEE, elbowUp = false)
        if (!ikNow.reachable) {
          phase = Phase.Error
          runtimeError = Some(ikNow.message)
        } else {
          qcmd = ikNow.q
        }
      }

      val fk = arm.forwardKinematics(qcmd)

      val approachRaw = fk.ee - fk.joint2
      val approach = {
        val n = approachRaw.norm
        if (n > 1e-6f) approachRaw * (1.0f / n) else Vec3(1f, 0f, 0f)
      }

      ballPos = ballState match {
        case BallState.AtStart   => start
        case BallState.AtGoal    => goal
        case BallState.Attached  => fk.ee + (approach * 0.22f)
      }

      Raylib.BeginDrawing()
      Raylib.ClearBackground(bg)

      Raylib.BeginMode3D(cam)

      // Thicker axes (no grid)
      val axisLen = 3.0f
      val axisR = 0.03f
      Raylib.DrawCylinderEx(DrawUtils.vec3(0f, 0f, 0f), DrawUtils.vec3(axisLen, 0f, 0f), axisR, axisR, 12, Colors.RED)
      Raylib.DrawCylinderEx(DrawUtils.vec3(0f, 0f, 0f), DrawUtils.vec3(0f, axisLen, 0f), axisR, axisR, 12, Colors.GREEN)
      Raylib.DrawCylinderEx(DrawUtils.vec3(0f, 0f, 0f), DrawUtils.vec3(0f, 0f, axisLen), axisR, axisR, 12, Colors.BLUE)

      // Robot visuals
      DrawUtils.drawRobotBasePedestal(Vec3.Zero)
      DrawUtils.drawRobotJointHousing(fk.base, 0.30f)
      DrawUtils.drawRobotJointHousing(fk.joint2, 0.24f)
      DrawUtils.drawRobotJointHousing(fk.ee, 0.18f)

      DrawUtils.drawTaperedLink(fk.base, fk.joint2, 0.14f, 0.12f, DrawUtils.color(185, 185, 190, 255))
      DrawUtils.drawTaperedLink(fk.joint2, fk.ee, 0.12f, 0.10f, DrawUtils.color(170, 170, 175, 255))

      DrawUtils.drawSuctionTool(fk.ee, approach)

      Raylib.DrawSphere(DrawUtils.vec3(ballPos), ballRadius, Colors.RED)
      Raylib.DrawSphereWires(DrawUtils.vec3(ballPos), ballRadius * 1.02f, 10, 10, Colors.RAYWHITE)

      Raylib.EndMode3D()

      val phaseText = phase match {
        case Phase.Idle             => "Phase: PAUSED (set START/GOAL, then PLAY)"
        case Phase.MoveHomeToStart  => "Phase: HOME -> START"
        case Phase.PickAtStart      => "Phase: PICK at START"
        case Phase.MoveStartToGoal  => "Phase: START -> GOAL (ball attached)"
        case Phase.PlaceAtGoal      => "Phase: PLACE at GOAL"
        case Phase.ReturnGoalToHome => "Phase: GOAL -> HOME"
        case Phase.WaitAtHomeReset  => "Phase: WAIT then LOOP"
        case Phase.Error            => "Phase: ERROR"
      }

      val status = Overlay.OverlayStatus(
        phaseText = phaseText,
        errorText = runtimeError.orElse {
          if (phase == Phase.Error) Some("ERROR: HOME/START/GOAL invalid or out of reach (z>=0 required).")
          else None
        }
      )

      val out = Overlay.drawOverlayPanel(
        font = uiFont,
        arm = arm,
        appliedStart = appliedStart,
        appliedGoal = appliedGoal,
        status = status,
        paused0 = paused,
        model0 = overlayModel,
        screenW = screenW,
        screenH = screenH
      )

      paused = out.paused
      overlayModel = out.model

      out.runRequest.foreach { rr =>
        val changed = !approxEq(rr.start, appliedStart) || !approxEq(rr.goal, appliedGoal)
        if (rr.reset || changed || phase == Phase.Error || phase == Phase.Idle) {
          resetSimulation(rr.start, rr.goal)
        }
        paused = false
      }

      DrawUtils.drawTextSmall(uiFont, uiHint, 12, screenH - 28, 18, DrawUtils.color(200, 200, 200, 220))

      Raylib.EndDrawing()
    }

    // Cleanup (avoid unloading the default font)
    try {
      val defTexId = Raylib.GetFontDefault().texture().id()
      if (uiFont.texture().id() != defTexId) Raylib.UnloadFont(uiFont)
    } catch { case _: Throwable => () }

    Raylib.CloseWindow()
  }
}
