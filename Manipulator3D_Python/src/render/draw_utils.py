from __future__ import annotations

from typing import Tuple

Vec3 = Tuple[float, float, float]


def _c(pr, color):
    """Normalize color to pyray Color."""
    if hasattr(color, "r") and hasattr(color, "g") and hasattr(color, "b") and hasattr(color, "a"):
        return color
    return pr.Color(int(color[0]), int(color[1]), int(color[2]), int(color[3]))


def _v2(pr, x: float, y: float):
    return pr.Vector2(float(x), float(y))


def _v3(pr, x: float, y: float, z: float):
    return pr.Vector3(float(x), float(y), float(z))


def _v3_add(a: Vec3, b: Vec3) -> Vec3:
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def _v3_scale(a: Vec3, s: float) -> Vec3:
    return (a[0] * s, a[1] * s, a[2] * s)


def draw_text_bold(pr, font, text: str, x: int, y: int, font_size: float, color) -> None:
    col = _c(pr, color)
    pr.draw_text_ex(font, text, _v2(pr, x, y), float(font_size), 1.0, col)
    pr.draw_text_ex(font, text, _v2(pr, x + 1, y), float(font_size), 1.0, col)
    pr.draw_text_ex(font, text, _v2(pr, x, y + 1), float(font_size), 1.0, col)
    pr.draw_text_ex(font, text, _v2(pr, x + 1, y + 1), float(font_size), 1.0, col)


def draw_text_small(pr, font, text: str, x: int, y: int, font_size: float, color) -> None:
    pr.draw_text_ex(font, text, _v2(pr, x, y), float(font_size), 1.0, _c(pr, color))


def draw_robot_base_pedestal(pr, origin: Vec3) -> None:
    ox, oy, _oz = origin
    a = _v3(pr, ox, oy, -0.25)
    b = _v3(pr, ox, oy, 0.00)
    pr.draw_cylinder_ex(a, b, 0.55, 0.55, 24, _c(pr, (70, 70, 75, 255)))

    c = _v3(pr, ox, oy, 0.00)
    d = _v3(pr, ox, oy, 0.35)
    pr.draw_cylinder_ex(c, d, 0.38, 0.34, 24, _c(pr, (95, 95, 100, 255)))

    pr.draw_cylinder_ex(_v3(pr, ox, oy, 0.00), _v3(pr, ox, oy, 0.06), 0.48, 0.48, 24, _c(pr, (110, 110, 115, 255)))


def draw_robot_joint_housing(pr, center: Vec3, radius: float) -> None:
    pr.draw_sphere(_v3(pr, center[0], center[1], center[2]), float(radius), _c(pr, (120, 120, 125, 255)))
    pr.draw_sphere_wires(_v3(pr, center[0], center[1], center[2]), float(radius), 12, 12, _c(pr, (200, 200, 200, 60)))

    pr.draw_cylinder_ex(
        _v3(pr, center[0], center[1], center[2] - 0.10),
        _v3(pr, center[0], center[1], center[2] + 0.10),
        float(radius * 0.55),
        float(radius * 0.55),
        18,
        _c(pr, (85, 85, 90, 255)),
    )


def draw_tapered_link(pr, a: Vec3, b: Vec3, r_a: float, r_b: float, color) -> None:
    pr.draw_cylinder_ex(_v3(pr, a[0], a[1], a[2]), _v3(pr, b[0], b[1], b[2]), float(r_a), float(r_b), 20, _c(pr, color))
    pr.draw_sphere(_v3(pr, a[0], a[1], a[2]), float(r_a * 0.95), _c(pr, (140, 140, 145, 255)))
    pr.draw_sphere(_v3(pr, b[0], b[1], b[2]), float(r_b * 0.95), _c(pr, (140, 140, 145, 255)))


def draw_suction_tool(pr, ee: Vec3, approach_dir: Vec3) -> None:
    tip = _v3_add(ee, _v3_scale(approach_dir, 0.28))
    pr.draw_cylinder_ex(
        _v3(pr, ee[0], ee[1], ee[2]),
        _v3(pr, tip[0], tip[1], tip[2]),
        0.06,
        0.05,
        18,
        _c(pr, (40, 40, 45, 255)),
    )

    cup_a = tip
    cup_b = _v3_add(tip, _v3_scale(approach_dir, 0.06))
    pr.draw_cylinder_ex(
        _v3(pr, cup_a[0], cup_a[1], cup_a[2]),
        _v3(pr, cup_b[0], cup_b[1], cup_b[2]),
        0.11,
        0.11,
        24,
        _c(pr, (25, 25, 28, 255)),
    )

    pr.draw_sphere(_v3(pr, tip[0], tip[1], tip[2]), 0.035, _c(pr, (80, 80, 85, 255)))
