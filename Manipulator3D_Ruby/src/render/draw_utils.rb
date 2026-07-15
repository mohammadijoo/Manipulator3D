# frozen_string_literal: true

require_relative "../raylib_bootstrap"
require_relative "../util/math3d"

module Render
  module_function

  def draw_text_ex_at(font, text, x, y, font_size, color)
    DrawTextEx(font, text.to_s, Vector2.create(x.to_f, y.to_f), font_size.to_f, 1.0, color)
  end

  def draw_text_bold(font, text, x, y, font_size, color)
    draw_text_ex_at(font, text, x, y, font_size, color)
    draw_text_ex_at(font, text, x + 1, y, font_size, color)
    draw_text_ex_at(font, text, x, y + 1, font_size, color)
    draw_text_ex_at(font, text, x + 1, y + 1, font_size, color)
  end

  def draw_text_small(font, text, x, y, font_size, color)
    draw_text_ex_at(font, text, x, y, font_size, color)
  end

  def draw_robot_base_pedestal(origin)
    a = Vector3.create(origin.x, origin.y, -0.25)
    b = Vector3.create(origin.x, origin.y, 0.00)
    DrawCylinderEx(a, b, 0.55, 0.55, 24, Color.from_u8(70, 70, 75, 255))

    c = Vector3.create(origin.x, origin.y, 0.00)
    d = Vector3.create(origin.x, origin.y, 0.35)
    DrawCylinderEx(c, d, 0.38, 0.34, 24, Color.from_u8(95, 95, 100, 255))

    DrawCylinderEx(
      Vector3.create(origin.x, origin.y, 0.00),
      Vector3.create(origin.x, origin.y, 0.06),
      0.48, 0.48, 24, Color.from_u8(110, 110, 115, 255)
    )
  end

  def draw_robot_joint_housing(center, radius)
    DrawSphere(center, radius, Color.from_u8(120, 120, 125, 255))
    DrawSphereWires(center, radius, 12, 12, Color.from_u8(200, 200, 200, 60))
    DrawCylinderEx(
      Vector3.create(center.x, center.y, center.z - 0.10),
      Vector3.create(center.x, center.y, center.z + 0.10),
      radius * 0.55, radius * 0.55, 18,
      Color.from_u8(85, 85, 90, 255)
    )
  end

  def draw_tapered_link(a, b, r_a, r_b, color)
    DrawCylinderEx(a, b, r_a, r_b, 20, color)
    DrawSphere(a, r_a * 0.95, Color.from_u8(140, 140, 145, 255))
    DrawSphere(b, r_b * 0.95, Color.from_u8(140, 140, 145, 255))
  end

  def draw_suction_tool(ee, approach_dir)
    tip = Vector3.create(ee.x + approach_dir.x * 0.28, ee.y + approach_dir.y * 0.28, ee.z + approach_dir.z * 0.28)
    DrawCylinderEx(ee, tip, 0.06, 0.05, 18, Color.from_u8(40, 40, 45, 255))

    cup_a = tip
    cup_b = Vector3.create(tip.x + approach_dir.x * 0.06, tip.y + approach_dir.y * 0.06, tip.z + approach_dir.z * 0.06)
    DrawCylinderEx(cup_a, cup_b, 0.11, 0.11, 24, Color.from_u8(25, 25, 28, 255))

    DrawSphere(tip, 0.035, Color.from_u8(80, 80, 85, 255))
  end
end
