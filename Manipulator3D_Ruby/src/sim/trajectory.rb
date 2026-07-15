# frozen_string_literal: true

require_relative "../raylib_bootstrap"
require_relative "../util/math3d"

module Sim
  class LinearTrajectory
    def initialize
      @a = Vector3.create(0.0, 0.0, 0.0)
      @b = Vector3.create(0.0, 0.0, 0.0)
      @duration = 1.0
      @t = 0.0
      @alpha = 0.0
      @finished = true
    end

    def reset(from, to, duration_sec)
      @a = Vector3.copy_from(from)
      @b = Vector3.copy_from(to)
      @duration = (duration_sec.to_f > 1e-6) ? duration_sec.to_f : 1e-6
      @t = 0.0
      @alpha = 0.0
      @finished = false
    end

    def update(dt)
      return if @finished
      @t += dt.to_f
      @alpha = Util::Math3D.clamp(@t / @duration, 0.0, 1.0)
      @finished = true if @alpha >= 1.0
    end

    def position
      Vector3.create(
        @a.x + (@b.x - @a.x) * @alpha,
        @a.y + (@b.y - @a.y) * @alpha,
        @a.z + (@b.z - @a.z) * @alpha
      )
    end

    def finished? = @finished
    def alpha = @alpha
  end
end
