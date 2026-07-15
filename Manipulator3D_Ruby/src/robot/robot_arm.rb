# frozen_string_literal: true

require_relative "../raylib_bootstrap"
require_relative "../util/math3d"

module Robot
  class LinkParams
    attr_accessor :length_m, :mass_kg, :inertia_cm, :inertia_joint

    def initialize(length_m: 2.5, mass_kg: 1.0)
      @length_m = length_m.to_f
      @mass_kg = mass_kg.to_f
      @inertia_cm = 0.0
      @inertia_joint = 0.0
      recompute_inertia
    end

    def recompute_inertia
      @inertia_cm = (1.0 / 12.0) * @mass_kg * @length_m * @length_m
      @inertia_joint = (1.0 / 3.0) * @mass_kg * @length_m * @length_m
    end
  end

  class JointAngles
    attr_accessor :q0_yaw, :q1_pitch, :q2_pitch

    def initialize(q0_yaw: 0.0, q1_pitch: 0.0, q2_pitch: 0.0)
      @q0_yaw = q0_yaw.to_f
      @q1_pitch = q1_pitch.to_f
      @q2_pitch = q2_pitch.to_f
    end

    def dup
      JointAngles.new(q0_yaw: @q0_yaw, q1_pitch: @q1_pitch, q2_pitch: @q2_pitch)
    end
  end

  class FKResult
    attr_accessor :base, :joint1, :joint2, :ee

    def initialize
      @base = Vector3.create(0.0, 0.0, 0.0)
      @joint1 = Vector3.create(0.0, 0.0, 0.0)
      @joint2 = Vector3.create(0.0, 0.0, 0.0)
      @ee = Vector3.create(0.0, 0.0, 0.0)
    end
  end

  class IKResult
    attr_accessor :reachable, :q, :message

    def initialize(reachable: false, q: JointAngles.new, message: "")
      @reachable = reachable
      @q = q
      @message = message
    end
  end

  class RobotArm
    def initialize(link1, link2)
      @link1 = link1
      @link2 = link2
      @link1.recompute_inertia
      @link2.recompute_inertia
    end

    attr_reader :link1, :link2

    def l1 = @link1.length_m
    def l2 = @link2.length_m

    def max_reach = l1 + l2
    def min_reach = (l1 - l2).abs

    def solve_ik(target, elbow_up: false)
      if target.z < 0.0
        return IKResult.new(reachable: false, message: "Invalid target: z must be >= 0")
      end

      x = target.x.to_f
      y = target.y.to_f
      z = target.z.to_f

      d = Math.sqrt(x * x + y * y + z * z)
      rmin = min_reach
      rmax = max_reach

      if d < rmin - 1e-9 || d > rmax + 1e-9
        msg = "Target radius |p|=#{d} is outside [#{rmin}, #{rmax}]"
        return IKResult.new(reachable: false, message: msg)
      end

      q0 = (x.abs > 1e-12 || y.abs > 1e-12) ? Math.atan2(y, x) : 0.0

      r = Math.sqrt(x * x + y * y)
      l1v = l1
      l2v = l2

      c2 = (r * r + z * z - l1v * l1v - l2v * l2v) / (2.0 * l1v * l2v)
      c2 = [[c2, -1.0].max, 1.0].min

      q2 = Math.acos(c2)
      q2 = -q2 unless elbow_up

      s2 = Math.sin(q2)
      k1 = l1v + l2v * Math.cos(q2)
      k2 = l2v * s2

      q1 = Math.atan2(z, r) - Math.atan2(k2, k1)

      IKResult.new(
        reachable: true,
        q: JointAngles.new(q0_yaw: q0, q1_pitch: q1, q2_pitch: q2),
        message: "OK"
      )
    end

    def forward_kinematics(q)
      fk = FKResult.new
      fk.base = Vector3.create(0.0, 0.0, 0.0)
      fk.joint1 = fk.base

      cy = Math.cos(q.q0_yaw)
      sy = Math.sin(q.q0_yaw)

      u = Vector3.create(cy, sy, 0.0)
      k = Vector3.create(0.0, 0.0, 1.0)

      p1 = Vector3.create(
        u.x * (l1 * Math.cos(q.q1_pitch)) + k.x * (l1 * Math.sin(q.q1_pitch)),
        u.y * (l1 * Math.cos(q.q1_pitch)) + k.y * (l1 * Math.sin(q.q1_pitch)),
        u.z * (l1 * Math.cos(q.q1_pitch)) + k.z * (l1 * Math.sin(q.q1_pitch))
      )

      a = q.q1_pitch + q.q2_pitch
      p2 = Vector3.create(
        p1.x + u.x * (l2 * Math.cos(a)) + k.x * (l2 * Math.sin(a)),
        p1.y + u.y * (l2 * Math.cos(a)) + k.y * (l2 * Math.sin(a)),
        p1.z + u.z * (l2 * Math.cos(a)) + k.z * (l2 * Math.sin(a))
      )

      fk.joint2 = p1
      fk.ee = p2
      fk
    end
  end
end
