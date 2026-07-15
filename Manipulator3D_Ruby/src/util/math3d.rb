# frozen_string_literal: true

require_relative "../raylib_bootstrap"

module Util
  module Math3D
    module_function

    def v2(x = 0.0, y = 0.0) = Vector2.create(x, y)
    def v3(x = 0.0, y = 0.0, z = 0.0) = Vector3.create(x, y, z)
    def rect(x = 0.0, y = 0.0, w = 0.0, h = 0.0) = Rectangle.create(x, y, w, h)
    def color_u8(r, g, b, a = 255) = Color.from_u8(r, g, b, a)

    def clamp(x, lo, hi)
      return lo if x < lo
      return hi if x > hi
      x
    end

    def add(a, b) = Vector3.create(a.x + b.x, a.y + b.y, a.z + b.z)
    def sub(a, b) = Vector3.create(a.x - b.x, a.y - b.y, a.z - b.z)
    def scale(v, s) = Vector3.create(v.x * s, v.y * s, v.z * s)

    def length(v)
      Math.sqrt(v.x * v.x + v.y * v.y + v.z * v.z)
    end

    def normalize(v)
      n = length(v)
      return Vector3.create(1.0, 0.0, 0.0) if n < 1e-9
      Vector3.create(v.x / n, v.y / n, v.z / n)
    end
  end
end
