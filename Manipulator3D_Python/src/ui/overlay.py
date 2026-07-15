from __future__ import annotations

from dataclasses import dataclass
from math import sqrt
from typing import Optional, Tuple

from robot.robot_arm import RobotArm
from pyray import RAYWHITE, SKYBLUE, GREEN, ORANGE, RED

Vec3 = Tuple[float, float, float]


@dataclass
class OverlayStatus:
    start_reachable: bool = False
    goal_reachable: bool = False
    error_text: Optional[str] = None
    phase_text: str = ""


def _norm3(v: Vec3) -> float:
    return sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2])


def _point_in_rect(p, r) -> bool:
    """
    Hit-test for mouse clicks.
    Supports pyray Vector2 and rect given as tuple (x,y,w,h) or pyray Rectangle.
    """
    px = p.x if hasattr(p, "x") else p[0]
    py = p.y if hasattr(p, "y") else p[1]

    rx = r.x if hasattr(r, "x") else r[0]
    ry = r.y if hasattr(r, "y") else r[1]
    rw = r.width if hasattr(r, "width") else r[2]
    rh = r.height if hasattr(r, "height") else r[3]

    return (px >= rx and px <= rx + rw and py >= ry and py <= ry + rh)


def _parse_vec3(text: str) -> Tuple[bool, Vec3, str]:
    parts = [p for p in text.replace(",", " ").split(" ") if p.strip() != ""]
    if len(parts) != 3:
        return False, (0.0, 0.0, 0.0), "Invalid format. Use: x y z"
    try:
        x, y, z = float(parts[0]), float(parts[1]), float(parts[2])
    except ValueError:
        return False, (0.0, 0.0, 0.0), "Invalid numbers. Use: x y z"
    if z < 0.0:
        return False, (x, y, z), "Invalid input: z must be >= 0"
    return True, (x, y, z), "OK"


_ALLOWED_CHARS = set("0123456789.-+ eE")


def _color(pr, c):
    """
    Normalize colors for pyray calls.
    Accepts:
      - pyray Color (has .r/.g/.b/.a)
      - [r,g,b,a] or (r,g,b,a)
    """
    if hasattr(c, "r") and hasattr(c, "g") and hasattr(c, "b") and hasattr(c, "a"):
        return c
    return pr.Color(int(c[0]), int(c[1]), int(c[2]), int(c[3]))


def _fit_text(pr, font, text: str, font_size: float, max_width_px: int) -> str:
    """
    Truncate with an ellipsis so the string stays inside the input box.
    Keeps the end of the string (better for editing).
    """
    if max_width_px <= 10:
        return ""

    size = pr.measure_text_ex(font, text, float(font_size), 1.0)
    if int(size.x) <= max_width_px:
        return text

    ell = "…"
    t = text
    while len(t) > 0:
        t = t[1:]
        size2 = pr.measure_text_ex(font, ell + t, float(font_size), 1.0)
        if int(size2.x) <= max_width_px:
            return ell + t
    return ell


class OverlayPanel:
    def __init__(self, start_default: Vec3, goal_default: Vec3) -> None:
        self.start_text = f"{start_default[0]} {start_default[1]} {start_default[2]}"
        self.goal_text = f"{goal_default[0]} {goal_default[1]} {goal_default[2]}"
        self._active: Optional[str] = None  # "start" | "goal" | None
        self._ui_error: Optional[str] = None

    def _handle_text_input(self, pr, text: str, max_len: int = 40) -> str:
        # Typed chars (raylib provides a per-frame queue)
        codepoint = pr.get_char_pressed()
        while codepoint > 0:
            ch = chr(codepoint)
            if ch in _ALLOWED_CHARS and len(text) < max_len:
                text += ch
            codepoint = pr.get_char_pressed()

        # Backspace
        if pr.is_key_pressed(pr.KEY_BACKSPACE) and len(text) > 0:
            text = text[:-1]

        # Escape/Enter ends editing
        if pr.is_key_pressed(pr.KEY_ESCAPE) or pr.is_key_pressed(pr.KEY_ENTER):
            self._active = None

        return text

    def draw(
        self,
        pr,
        font,
        arm: RobotArm,
        start: Vec3,
        goal: Vec3,
        status: OverlayStatus,
        paused: bool,
        screen_w: int,
        screen_h: int,
    ) -> Tuple[bool, Vec3, Vec3, bool]:
        """
        Returns:
            paused (bool): updated pause state
            start (Vec3): updated start target (only changes when PLAY succeeds)
            goal  (Vec3): updated goal target  (only changes when PLAY succeeds)
            restart (bool): True if a new simulation should start using updated targets
        """
        restart = False

        from render.draw_utils import draw_text_bold, draw_text_small

        # Panel geometry
        pad = 12
        x0, y0 = 14, 14
        w, h = 360, 520

        # Stronger contrast so the panel & inputs are clearly visible
        panel_fill = _color(pr, (22, 22, 24, 235))
        panel_border = _color(pr, (230, 230, 235, 255))

        pr.draw_rectangle(x0, y0, w, h, panel_fill)
        pr.draw_rectangle_lines(x0, y0, w, h, panel_border)

        # Spacing tuned to avoid overlaps
        line18 = 22
        line22 = 28
        line24 = 30

        y = y0 + pad

        # Title
        draw_text_bold(pr, font, "3-DOF 2-Link Arm", x0 + pad, y, 24, RAYWHITE)
        y += line24

        # Phase
        if status.phase_text:
            draw_text_small(pr, font, status.phase_text, x0 + pad, y, 18, SKYBLUE)
            y += line18

        # Play/Pause button (plain rect for maximum compatibility)
        btn = (float(x0 + pad), float(y), float(w - 2 * pad), 34.0)
        btn_bg = (70, 140, 70, 230) if paused else (150, 70, 70, 230)
        pr.draw_rectangle(int(btn[0]), int(btn[1]), int(btn[2]), int(btn[3]), _color(pr, btn_bg))
        pr.draw_rectangle_lines(int(btn[0]), int(btn[1]), int(btn[2]), int(btn[3]), _color(pr, (245, 245, 245, 255)))

        label = "PLAY" if paused else "PAUSE"
        draw_text_bold(pr, font, label, int(btn[0]) + 10, int(btn[1]) + 6, 22, RAYWHITE)

        mp = pr.get_mouse_position()
        if pr.is_mouse_button_pressed(pr.MOUSE_BUTTON_LEFT) and _point_in_rect(mp, btn):
            if paused:
                ok_s, v_s, msg_s = _parse_vec3(self.start_text)
                ok_g, v_g, msg_g = _parse_vec3(self.goal_text)

                if not ok_s:
                    self._ui_error = f"START: {msg_s}"
                elif not ok_g:
                    self._ui_error = f"GOAL: {msg_g}"
                else:
                    ik_s = arm.solve_ik(v_s, elbow_up=False)
                    ik_g = arm.solve_ik(v_g, elbow_up=False)

                    if not ik_s.reachable:
                        self._ui_error = f"START not reachable: {ik_s.message}"
                    elif not ik_g.reachable:
                        self._ui_error = f"GOAL not reachable: {ik_g.message}"
                    else:
                        self._ui_error = None
                        start, goal = v_s, v_g
                        paused = False
                        restart = True
                        self._active = None
            else:
                paused = True
                self._active = None

        y += 46

        # Input fields (editable only when paused)
        base_box_h = 30
        box_h = int(base_box_h * 1.1)  # +15%
        box_w = w - 2 * pad

        label_gap_top = 6    # space above label
        label_gap_bottom = 6 # space between label and box
        box_gap = 14         # space between boxes

        # START label + box
        y += label_gap_top
        draw_text_small(pr, font, "START (x y z):", x0 + pad, y, 18, RAYWHITE)
        y += 18 + label_gap_bottom
        start_rect = (float(x0 + pad), float(y), float(box_w), float(box_h))
        y += box_h + box_gap

        # GOAL label + box
        draw_text_small(pr, font, "GOAL  (x y z):", x0 + pad, y, 18, RAYWHITE)
        y += 18 + label_gap_bottom
        goal_rect = (float(x0 + pad), float(y), float(box_w), float(box_h))
        y += box_h + 16

        def _draw_input_box(rect, text: str, active: bool) -> None:
            rx, ry, rw, rh = int(rect[0]), int(rect[1]), int(rect[2]), int(rect[3])

            fill = (40, 40, 46, 245) if paused else (32, 32, 36, 235)
            border = (255, 255, 255, 255) if (paused and active) else (210, 210, 215, 230)

            pr.draw_rectangle(rx, ry, rw, rh, _color(pr, fill))
            pr.draw_rectangle_lines(rx, ry, rw, rh, _color(pr, border))

            # Fit text to box width
            inner_w = rw - 16
            shown = _fit_text(pr, font, text, 18.0, max(inner_w, 10))

            # Vertically center text (prevents overflow and adds top/bottom padding)
            text_h = int(pr.measure_text_ex(font, "Ag", 18.0, 1.0).y)
            ty = ry + max(4, (rh - text_h) // 2)

            draw_text_small(pr, font, shown, rx + 8, ty, 18, RAYWHITE)

            # Caret for active field (paused only)
            if paused and active:
                caret_w = int(pr.measure_text_ex(font, shown, 18.0, 1.0).x)
                cx = rx + 8 + caret_w + 1
                cy0 = ty
                cy1 = ty + text_h
                pr.draw_line(cx, cy0, cx, cy1, _color(pr, (250, 250, 250, 230)))


        _draw_input_box(start_rect, self.start_text, self._active == "start")
        _draw_input_box(goal_rect, self.goal_text, self._active == "goal")

        # Activate fields by click (only when paused)
        if paused and pr.is_mouse_button_pressed(pr.MOUSE_BUTTON_LEFT):
            if _point_in_rect(mp, start_rect):
                self._active = "start"
            elif _point_in_rect(mp, goal_rect):
                self._active = "goal"
            else:
                self._active = None

        # Text input (only when paused)
        if paused and self._active == "start":
            self.start_text = self._handle_text_input(pr, self.start_text)
        if paused and self._active == "goal":
            self.goal_text = self._handle_text_input(pr, self.goal_text)

        # Link parameters and workspace
        L1 = arm.link1
        L2 = arm.link2

        buf = f"Link1 length: {L1.length_m:.3f}m"
        draw_text_small(pr, font, buf, x0 + pad, y, 18, RAYWHITE); y += 20
        buf = f"Link1 mass  : {L1.mass_kg:.3f}kg"
        draw_text_small(pr, font, buf, x0 + pad, y, 18, RAYWHITE); y += 20
        buf = f"Link1 inertia (joint): {L1.inertia_joint:.5f}"
        draw_text_small(pr, font, buf, x0 + pad, y, 18, RAYWHITE); y += 24

        buf = f"Link2 length: {L2.length_m:.3f}m"
        draw_text_small(pr, font, buf, x0 + pad, y, 18, RAYWHITE); y += 20
        buf = f"Link2 mass  : {L2.mass_kg:.3f}kg"
        draw_text_small(pr, font, buf, x0 + pad, y, 18, RAYWHITE); y += 20
        buf = f"Link2 inertia (joint): {L2.inertia_joint:.5f}"
        draw_text_small(pr, font, buf, x0 + pad, y, 18, RAYWHITE); y += 26

        rmin = arm.min_reach
        rmax = arm.max_reach
        buf = f"Workspace |p|: [{rmin:.2f}, {rmax:.2f}]m"
        draw_text_small(pr, font, buf, x0 + pad, y, 18, _color(pr, (140, 200, 255, 255))); y += 26

        # Reachability feedback (based on current live targets)
        ds = _norm3(start)
        dg = _norm3(goal)

        buf = f"Start |p|={ds:.3f}"
        draw_text_small(pr, font, buf, x0 + pad, y, 18, GREEN if status.start_reachable else ORANGE); y += 20
        buf = f"Start: ({start[0]:.2f}, {start[1]:.2f}, {start[2]:.2f})"
        draw_text_small(pr, font, buf, x0 + pad, y, 18, GREEN if status.start_reachable else ORANGE); y += 24

        buf = f"Goal  |p|={dg:.3f}"
        draw_text_small(pr, font, buf, x0 + pad, y, 18, GREEN if status.goal_reachable else ORANGE); y += 20
        buf = f"Goal : ({goal[0]:.2f}, {goal[1]:.2f}, {goal[2]:.2f})"
        draw_text_small(pr, font, buf, x0 + pad, y, 18, GREEN if status.goal_reachable else ORANGE); y += 26

        if not status.start_reachable or not status.goal_reachable:
            draw_text_bold(pr, font, "OUT OF REACH!", x0 + pad, y, 22, RED)
            y += line22
            draw_text_small(pr, font, "Choose points inside workspace.", x0 + pad, y, 18, RED)
            y += line18

        # Errors (runtime first, then UI validation)
        err = status.error_text or self._ui_error
        if err:
            # Keep error readable; truncate if very long
            maxw = (w - 2 * pad)
            shown_err = _fit_text(pr, font, err, 18.0, maxw)
            draw_text_bold(pr, font, shown_err, x0 + pad, y, 18, RED)

        # Help line (bottom-right of screen)
        hint = "Mouse Wheel: zoom   F11: fullscreen"
        size = pr.measure_text_ex(font, hint, 18.0, 1.0)
        tw = int(size.x)
        draw_text_small(pr, font, hint, int(screen_w - tw - 12), int(screen_h - 28), 18, _color(pr, (200, 200, 200, 220)))

        return paused, start, goal, restart
