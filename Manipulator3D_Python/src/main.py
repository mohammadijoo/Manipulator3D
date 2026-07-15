from __future__ import annotations

import os
from enum import Enum, auto
from math import sqrt
from typing import Optional, Tuple

import pyray as pr
from pyray import BLUE, GREEN, ORANGE, RAYWHITE, RED

from robot.robot_arm import JointAngles, LinkParams, RobotArm
from sim.trajectory import LinearTrajectory
from ui.overlay import OverlayPanel, OverlayStatus
from render import draw_utils as draw


Vec3 = Tuple[float, float, float]


def _clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


def _v3_add(a: Vec3, b: Vec3) -> Vec3:
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def _v3_sub(a: Vec3, b: Vec3) -> Vec3:
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def _v3_scale(a: Vec3, s: float) -> Vec3:
    return (a[0] * s, a[1] * s, a[2] * s)


def _v3_len(a: Vec3) -> float:
    return sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2])


def _v3_norm(a: Vec3) -> Vec3:
    l = _v3_len(a)
    if l <= 1e-9:
        return (1.0, 0.0, 0.0)
    return (a[0] / l, a[1] / l, a[2] / l)


def _load_best_ui_font(pr, px: int):
    candidates = [
        os.path.join("resources", "fonts", "Inter-Regular.ttf"),
        os.path.join("..", "resources", "fonts", "Inter-Regular.ttf"),
        r"C:\Windows\Fonts\segoeui.ttf",
        r"C:\Windows\Fonts\arial.ttf",
        "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
    ]

    for path in candidates:
        if os.path.exists(path):
            try:
                f = pr.load_font_ex(path, px, None, 0)
                tex = getattr(f, "texture", None)
                if tex is not None and getattr(tex, "id", 0) != 0:
                    pr.set_texture_filter(tex, pr.TEXTURE_FILTER_BILINEAR)
                    return f
            except Exception:
                pass

    f = pr.get_font_default()
    pr.set_texture_filter(f.texture, pr.TEXTURE_FILTER_BILINEAR)
    return f

def _update_zoom(pr, cam) -> None:
    wheel = pr.get_mouse_wheel_move()
    if wheel == 0.0:
        return

    # Vector from target to camera position
    vx = float(cam.position.x - cam.target.x)
    vy = float(cam.position.y - cam.target.y)
    vz = float(cam.position.z - cam.target.z)

    dist0 = (vx * vx + vy * vy + vz * vz) ** 0.5
    if dist0 < 1e-3:
        dist0 = 1e-3

    scale = 1.0 - float(wheel) * 0.10
    if scale < 0.70:
        scale = 0.70
    if scale > 1.30:
        scale = 1.30

    dist = dist0 * scale
    if dist < 1.0:
        dist = 1.0
    if dist > 200.0:
        dist = 200.0

    # Normalize direction (target->position) using original distance
    inv = 1.0 / dist0
    dx, dy, dz = vx * inv, vy * inv, vz * inv

    cam.position.x = cam.target.x + dx * dist
    cam.position.y = cam.target.y + dy * dist
    cam.position.z = cam.target.z + dz * dist


class Phase(Enum):
    MOVE_HOME_TO_START = auto()
    PICK_AT_START = auto()
    MOVE_START_TO_GOAL = auto()
    PLACE_AT_GOAL = auto()
    RETURN_GOAL_TO_HOME = auto()
    WAIT_AT_HOME_RESET = auto()
    ERROR = auto()
    IDLE = auto()


class BallState(Enum):
    AT_START = auto()
    ATTACHED = auto()
    AT_GOAL = auto()


def main() -> None:
    pr.set_trace_log_level(pr.LOG_ERROR)
    pr.set_config_flags(pr.FLAG_WINDOW_RESIZABLE | pr.FLAG_MSAA_4X_HINT)

    # --- Arm parameters ---
    link1 = LinkParams(length_m=3.0, mass_kg=2.0)
    link1.recompute_inertia()
    link2 = LinkParams(length_m=2.6, mass_kg=1.6)
    link2.recompute_inertia()

    arm = RobotArm(link1, link2)

    # Defaults (editable in the overlay)
    start: Vec3 = (1.0, 2.0, 1.0)
    goal: Vec3 = (2.0, 3.0, 2.0)

    # Fixed EE "home" position
    home_ee: Vec3 = (2.0, 2.0, 2.0)

    # Window / camera
    pr.init_window(1280, 720, "Manipulator3D - 3DOF IK Pick&Place (Python)")
    pr.set_window_min_size(960, 540)
    pr.set_target_fps(60)

    ui_font = _load_best_ui_font(pr, 22)

    reach = arm.max_reach
    cam = pr.Camera3D(
        [1.10 * reach, -1.15 * reach, 0.85 * reach],
        [0.0, 0.0, 0.35 * reach],
        [0.0, 0.0, 1.0],
        52.0,
        pr.CAMERA_PERSPECTIVE,
    )

    # Overlay UI
    overlay = OverlayPanel(start_default=start, goal_default=goal)

    # Timing
    move_home_to_start = 2.2
    pick_duration = 0.45
    move_start_to_goal = 2.6
    place_duration = 0.35
    return_to_home = 2.0
    reset_wait_total = 1.5

    # State variables
    paused = True
    phase = Phase.IDLE
    timer = 0.0
    runtime_error: Optional[str] = None

    # Ball
    ball_radius = _clamp(0.03 * reach, 0.06, 0.16)
    ball_state = BallState.AT_START
    ball_pos: Vec3 = start

    # Trajectory + targets
    traj = LinearTrajectory()
    target_ee: Vec3 = home_ee

    # Joint command
    ik_home = arm.solve_ik(home_ee, elbow_up=False)
    if not ik_home.reachable:
        pr.close_window()
        raise RuntimeError(f"HOME is not reachable: {ik_home.message}")

    qcmd = ik_home.q

    def reset_simulation(new_start: Vec3, new_goal: Vec3) -> bool:
        nonlocal start, goal, phase, timer, runtime_error, ball_state, ball_pos, target_ee, qcmd

        start = new_start
        goal = new_goal
        runtime_error = None

        ik_s = arm.solve_ik(start, elbow_up=False)
        ik_g = arm.solve_ik(goal, elbow_up=False)
        if not ik_s.reachable or not ik_g.reachable:
            phase = Phase.ERROR
            runtime_error = "ERROR: HOME/START/GOAL invalid or out of reach (z>=0 required)."
            return False

        qcmd = ik_home.q
        target_ee = home_ee

        ball_state = BallState.AT_START
        ball_pos = start

        traj.reset(home_ee, start, move_home_to_start)
        phase = Phase.MOVE_HOME_TO_START
        timer = 0.0
        return True

    while not pr.window_should_close():
        if pr.is_key_pressed(pr.KEY_F11):
            pr.toggle_fullscreen()

        screen_w = pr.get_screen_width()
        screen_h = pr.get_screen_height()

        _update_zoom(pr, cam)

        dt = 0.0 if paused else pr.get_frame_time()

        # Reachability (for display)
        ik_s_disp = arm.solve_ik(start, elbow_up=False)
        ik_g_disp = arm.solve_ik(goal, elbow_up=False)

        status = OverlayStatus(
            start_reachable=ik_s_disp.reachable,
            goal_reachable=ik_g_disp.reachable,
            error_text=runtime_error,
            phase_text="",
        )

        if phase == Phase.MOVE_HOME_TO_START:
            status.phase_text = "Phase: HOME -> START"
        elif phase == Phase.PICK_AT_START:
            status.phase_text = "Phase: PICK at START"
        elif phase == Phase.MOVE_START_TO_GOAL:
            status.phase_text = "Phase: START -> GOAL (ball attached)"
        elif phase == Phase.PLACE_AT_GOAL:
            status.phase_text = "Phase: PLACE at GOAL"
        elif phase == Phase.RETURN_GOAL_TO_HOME:
            status.phase_text = "Phase: GOAL -> HOME"
        elif phase == Phase.WAIT_AT_HOME_RESET:
            status.phase_text = "Phase: WAIT then LOOP"
        elif phase == Phase.ERROR:
            status.phase_text = "Phase: ERROR"
        else:
            status.phase_text = "Phase: PAUSED (edit START/GOAL and press PLAY)" if paused else "Phase: READY"

        # UI panel (may request restart)
        paused, start, goal, restart = overlay.draw(pr, ui_font, arm, start, goal, status, paused, screen_w, screen_h)
        if restart:
            # If restart requested, reset the simulation state immediately.
            ok = reset_simulation(start, goal)
            if not ok:
                paused = True

        # Simulation update (FSM)
        if phase not in (Phase.ERROR, Phase.IDLE) and not paused:
            if phase == Phase.MOVE_HOME_TO_START:
                ball_state = BallState.AT_START
                ball_pos = start

                traj.update(dt)
                target_ee = traj.position()

                if traj.finished:
                    phase = Phase.PICK_AT_START
                    timer = 0.0
                    target_ee = start

            elif phase == Phase.PICK_AT_START:
                target_ee = start
                ball_state = BallState.AT_START
                ball_pos = start

                timer += dt
                if timer >= pick_duration:
                    ball_state = BallState.ATTACHED
                    timer = 0.0
                    traj.reset(start, goal, move_start_to_goal)
                    phase = Phase.MOVE_START_TO_GOAL

            elif phase == Phase.MOVE_START_TO_GOAL:
                traj.update(dt)
                target_ee = traj.position()
                ball_state = BallState.ATTACHED

                if traj.finished:
                    phase = Phase.PLACE_AT_GOAL
                    timer = 0.0
                    target_ee = goal

            elif phase == Phase.PLACE_AT_GOAL:
                target_ee = goal

                timer += dt
                if timer >= place_duration:
                    ball_state = BallState.AT_GOAL
                    timer = 0.0  # time since place
                    traj.reset(goal, home_ee, return_to_home)
                    phase = Phase.RETURN_GOAL_TO_HOME
                else:
                    ball_state = BallState.ATTACHED

            elif phase == Phase.RETURN_GOAL_TO_HOME:
                ball_state = BallState.AT_GOAL

                traj.update(dt)
                target_ee = traj.position()

                timer += dt  # time since place
                if traj.finished:
                    phase = Phase.WAIT_AT_HOME_RESET

            elif phase == Phase.WAIT_AT_HOME_RESET:
                target_ee = home_ee
                timer += dt

                if timer >= reset_wait_total:
                    ball_state = BallState.AT_START
                    ball_pos = start

                    timer = 0.0
                    traj.reset(home_ee, start, move_home_to_start)
                    phase = Phase.MOVE_HOME_TO_START
                else:
                    ball_state = BallState.AT_GOAL

            # IK for current EE target
            ik_now = arm.solve_ik(target_ee, elbow_up=False)
            if not ik_now.reachable:
                phase = Phase.ERROR
                runtime_error = ik_now.message
                paused = True
            else:
                qcmd = ik_now.q

        # Forward kinematics for rendering
        fk = arm.forward_kinematics(qcmd)

        # Tool approach direction
        approach = _v3_sub(fk.ee, fk.joint2)
        approach = _v3_norm(approach)

        # Ball position by state (always visible)
        if ball_state == BallState.AT_START:
            ball_pos = start
        elif ball_state == BallState.AT_GOAL:
            ball_pos = goal
        else:
            ball_pos = _v3_add(fk.ee, _v3_scale(approach, 0.22))

        # Draw
        pr.begin_drawing()
        pr.clear_background([10, 12, 16, 255])

        pr.begin_mode_3d(cam)

        # Thicker axes using cylinders
        axis_len = 3.0
        axis_r = 0.03
        pr.draw_cylinder_ex([0.0, 0.0, 0.0], [axis_len, 0.0, 0.0], axis_r, axis_r, 12, RED)
        pr.draw_cylinder_ex([0.0, 0.0, 0.0], [0.0, axis_len, 0.0], axis_r, axis_r, 12, GREEN)
        pr.draw_cylinder_ex([0.0, 0.0, 0.0], [0.0, 0.0, axis_len], axis_r, axis_r, 12, BLUE)

        # Robot visuals
        draw.draw_robot_base_pedestal(pr, (0.0, 0.0, 0.0))
        draw.draw_robot_joint_housing(pr, fk.base, 0.30)
        draw.draw_robot_joint_housing(pr, fk.joint2, 0.24)
        draw.draw_robot_joint_housing(pr, fk.ee, 0.18)

        draw.draw_tapered_link(pr, fk.base, fk.joint2, 0.14, 0.12, [185, 185, 190, 255])
        draw.draw_tapered_link(pr, fk.joint2, fk.ee, 0.12, 0.10, [170, 170, 175, 255])

        draw.draw_suction_tool(pr, fk.ee, approach)

        # Ball (smaller + outline)
        pr.draw_sphere(list(ball_pos), ball_radius, RED)
        pr.draw_sphere_wires(list(ball_pos), ball_radius * 1.02, 10, 10, RAYWHITE)

        pr.end_mode_3d()

        pr.end_drawing()

    if getattr(ui_font.texture, "id", 0) != getattr(pr.get_font_default().texture, "id", 0):
        pr.unload_font(ui_font)
    pr.close_window()


if __name__ == "__main__":
    main()
