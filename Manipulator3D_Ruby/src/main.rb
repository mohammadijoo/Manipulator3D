# frozen_string_literal: true

require_relative "raylib_bootstrap"

require_relative "util/math3d"
require_relative "robot/robot_arm"
require_relative "sim/trajectory"
require_relative "render/draw_utils"
require_relative "ui/overlay"

include Util::Math3D

module App
  module_function

  def try_font(path, px)
    return nil unless FileExists(path) || File.exist?(path)

    f = LoadFontEx(path, px, nil, 0)
    begin
      SetTextureFilter(f.texture, TEXTURE_FILTER_BILINEAR)
    rescue StandardError
      # texture filter may not be available on some builds
    end
    f
  rescue StandardError
    nil
  end

  def load_best_ui_font(px)
    candidates = [
      "resources/fonts/Inter-Regular.ttf",
      "../resources/fonts/Inter-Regular.ttf",
      "C:/Windows/Fonts/segoeui.ttf",
      "C:/Windows/Fonts/arial.ttf"
    ]

    candidates.each do |path|
      f = try_font(path, px)
      return f if f
    end

    f = GetFontDefault()
    begin
      SetTextureFilter(f.texture, TEXTURE_FILTER_BILINEAR)
    rescue StandardError
    end
    f
  end

  def update_zoom(camera)
    wheel = GetMouseWheelMove()
    return if wheel == 0.0

    v = Util::Math3D.sub(camera.position, camera.target)
    dist = Util::Math3D.length(v)
    dist = 0.001 if dist < 0.001

    scale = 1.0 - wheel * 0.10
    scale = Util::Math3D.clamp(scale, 0.70, 1.30)

    dist *= scale
    dist = Util::Math3D.clamp(dist, 1.0, 200.0)

    dir = Util::Math3D.normalize(v)
    camera.position.set(
      camera.target.x + dir.x * dist,
      camera.target.y + dir.y * dist,
      camera.target.z + dir.z * dist
    )
  end

  def phase_text(phase)
    case phase
    when :idle                then "Phase: IDLE (set START/GOAL then PLAY)"
    when :move_home_to_start  then "Phase: HOME -> START"
    when :pick_at_start       then "Phase: PICK at START"
    when :move_start_to_goal  then "Phase: START -> GOAL (ball attached)"
    when :place_at_goal       then "Phase: PLACE at GOAL"
    when :return_goal_to_home then "Phase: GOAL -> HOME"
    when :wait_at_home_reset  then "Phase: WAIT then LOOP"
    when :error               then "Phase: ERROR"
    else "Phase: ?"
    end
  end

  def run
    begin
      SetTraceLogLevel(LOG_ERROR)
    rescue StandardError
    end

    begin
      SetConfigFlags(FLAG_WINDOW_RESIZABLE | FLAG_MSAA_4X_HINT)
    rescue StandardError
    end

    link1 = Robot::LinkParams.new(length_m: 3.0, mass_kg: 2.0)
    link2 = Robot::LinkParams.new(length_m: 2.6, mass_kg: 1.6)
    arm = Robot::RobotArm.new(link1, link2)

    start = Vector3.create(1.0, 2.0, 1.0)
    goal  = Vector3.create(2.0, 3.0, 2.0)
    home_ee = Vector3.create(2.0, 2.0, 2.0)

    InitWindow(1280, 720, "Manipulator3D - 3DOF IK Pick&Place (Ruby)")
    begin
      SetWindowMinSize(960, 540)
    rescue StandardError
    end
    SetTargetFPS(60)

    ui_font = load_best_ui_font(22)

    reach = arm.max_reach
    cam = Camera.new
    cam.target.set(0.0, 0.0, 0.35 * reach)
    cam.up.set(0.0, 0.0, 1.0)
    cam.fovy = 52.0
    cam.projection = CAMERA_PERSPECTIVE
    cam.position.set(1.10 * reach, -1.15 * reach, 0.85 * reach)

    overlay = UI::Overlay.new(ui_font)
    overlay.set_fields_from_vectors(start, goal)

    paused = true
    phase = :idle

    ball_state = :at_start
    ball_pos = Vector3.copy_from(start)

    target_ee = Vector3.copy_from(home_ee)
    qcmd = Robot::JointAngles.new
    traj = Sim::LinearTrajectory.new
    timer = 0.0

    move_home_to_start = 2.2
    pick_duration = 0.45
    move_start_to_goal = 2.6
    place_duration = 0.35
    return_to_home = 2.0
    reset_wait_total = 1.5

    ik_home = arm.solve_ik(home_ee, elbow_up: false)
    if ik_home.reachable
      qcmd = ik_home.q.dup
    else
      phase = :error
      paused = true
    end

    runtime_error = nil

    until WindowShouldClose()
      ToggleFullscreen() if IsKeyPressed(KEY_F11)

      screen_w = GetScreenWidth()
      screen_h = GetScreenHeight()

      update_zoom(cam)

      dt = paused ? 0.0 : GetFrameTime()
      runtime_error = nil

      if phase != :error && !paused
        case phase
        when :move_home_to_start
          ball_state = :at_start
          ball_pos = Vector3.copy_from(start)

          traj.update(dt)
          target_ee = traj.position

          if traj.finished?
            phase = :pick_at_start
            timer = 0.0
            target_ee = Vector3.copy_from(start)
          end

        when :pick_at_start
          target_ee = Vector3.copy_from(start)
          ball_state = :at_start
          ball_pos = Vector3.copy_from(start)

          timer += dt
          if timer >= pick_duration
            ball_state = :attached
            timer = 0.0
            traj.reset(start, goal, move_start_to_goal)
            phase = :move_start_to_goal
          end

        when :move_start_to_goal
          traj.update(dt)
          target_ee = traj.position
          ball_state = :attached

          if traj.finished?
            phase = :place_at_goal
            timer = 0.0
            target_ee = Vector3.copy_from(goal)
          end

        when :place_at_goal
          target_ee = Vector3.copy_from(goal)
          timer += dt
          if timer >= place_duration
            ball_state = :at_goal
            timer = 0.0
            traj.reset(goal, home_ee, return_to_home)
            phase = :return_goal_to_home
          else
            ball_state = :attached
          end

        when :return_goal_to_home
          ball_state = :at_goal
          traj.update(dt)
          target_ee = traj.position
          timer += dt
          phase = :wait_at_home_reset if traj.finished?

        when :wait_at_home_reset
          target_ee = Vector3.copy_from(home_ee)
          timer += dt
          if timer >= reset_wait_total
            ball_state = :at_start
            ball_pos = Vector3.copy_from(start)
            timer = 0.0
            traj.reset(home_ee, start, move_home_to_start)
            phase = :move_home_to_start
          else
            ball_state = :at_goal
          end
        end
      end

      if phase != :error
        ik_now = arm.solve_ik(target_ee, elbow_up: false)
        if ik_now.reachable
          qcmd = ik_now.q.dup
        else
          phase = :error
          runtime_error = ik_now.message
          paused = true
        end
      end

      fk = arm.forward_kinematics(qcmd)
      approach = Util::Math3D.normalize(Util::Math3D.sub(fk.ee, fk.joint2))

      case ball_state
      when :at_start
        ball_pos = Vector3.copy_from(start)
      when :at_goal
        ball_pos = Vector3.copy_from(goal)
      when :attached
        ball_pos = Vector3.create(
          fk.ee.x + approach.x * 0.22,
          fk.ee.y + approach.y * 0.22,
          fk.ee.z + approach.z * 0.22
        )
      end

      ball_radius = Util::Math3D.clamp(0.03 * reach, 0.06, 0.16)

      BeginDrawing()
      ClearBackground(Color.from_u8(10, 12, 16, 255))

      BeginMode3D(cam)

      axis_len = 3.0
      axis_r = 0.03
      DrawCylinderEx(Vector3.create(0, 0, 0), Vector3.create(axis_len, 0, 0), axis_r, axis_r, 12, RED)
      DrawCylinderEx(Vector3.create(0, 0, 0), Vector3.create(0, axis_len, 0), axis_r, axis_r, 12, GREEN)
      DrawCylinderEx(Vector3.create(0, 0, 0), Vector3.create(0, 0, axis_len), axis_r, axis_r, 12, BLUE)

      Render.draw_robot_base_pedestal(Vector3.create(0, 0, 0))
      Render.draw_robot_joint_housing(fk.base, 0.30)
      Render.draw_robot_joint_housing(fk.joint2, 0.24)
      Render.draw_robot_joint_housing(fk.ee, 0.18)

      Render.draw_tapered_link(fk.base, fk.joint2, 0.14, 0.12, Color.from_u8(185, 185, 190, 255))
      Render.draw_tapered_link(fk.joint2, fk.ee, 0.12, 0.10, Color.from_u8(170, 170, 175, 255))

      Render.draw_suction_tool(fk.ee, approach)

      DrawSphere(ball_pos, ball_radius, RED)
      DrawSphereWires(ball_pos, ball_radius * 1.02, 10, 10, RAYWHITE)

      EndMode3D()

      overlay_result = overlay.draw(
        arm: arm,
        paused: paused,
        start_current: start,
        goal_current: goal,
        phase_text: phase_text(phase),
        runtime_error: (phase == :error && runtime_error.nil?) ? "ERROR: HOME/START/GOAL invalid or out of reach (z>=0 required)." : runtime_error,
        screen_w: screen_w,
        screen_h: screen_h
      )

      paused = overlay_result[:paused]

      if overlay_result[:apply]
        if overlay_result[:apply][:ok]
          start = overlay_result[:apply][:start]
          goal  = overlay_result[:apply][:goal]

          ik_home = arm.solve_ik(home_ee, elbow_up: false)
          ik_start = arm.solve_ik(start, elbow_up: false)
          ik_goal  = arm.solve_ik(goal,  elbow_up: false)

          if ik_home.reachable && ik_start.reachable && ik_goal.reachable
            qcmd = ik_home.q.dup
            target_ee = Vector3.copy_from(home_ee)

            ball_state = :at_start
            ball_pos = Vector3.copy_from(start)

            traj.reset(home_ee, start, move_home_to_start)
            phase = :move_home_to_start
            timer = 0.0
            paused = false
          else
            phase = :error
            runtime_error = "ERROR: HOME/START/GOAL invalid or out of reach (z>=0 required)."
            paused = true
          end
        else
          phase = :error
          runtime_error = overlay_result[:apply][:message] || "Invalid inputs."
          paused = true
        end
      end

      Render.draw_text_small(ui_font, "F11: fullscreen   Mouse Wheel: zoom", 12, screen_h - 28, 18, Color.from_u8(200, 200, 200, 220))

      EndDrawing()
    end

    CloseWindow()
  end
end

App.run
