from __future__ import annotations

from dataclasses import dataclass
from math import acos, atan2, cos, fabs, sin, sqrt
from typing import Tuple


Vec3 = Tuple[float, float, float]


@dataclass
class LinkParams:
    length_m: float = 2.5
    mass_kg: float = 1.0

    # Uniform rod approximations:
    # I_cm    = (1/12) m L^2   (about center, axis ⟂ to rod)
    # I_joint = (1/3)  m L^2   (about joint at one end)
    inertia_cm: float = 0.0
    inertia_joint: float = 0.0

    def recompute_inertia(self) -> None:
        self.inertia_cm = (1.0 / 12.0) * self.mass_kg * self.length_m * self.length_m
        self.inertia_joint = (1.0 / 3.0) * self.mass_kg * self.length_m * self.length_m


@dataclass
class JointAngles:
    q0_yaw: float = 0.0     # rad
    q1_pitch: float = 0.0   # rad (relative to XY plane)
    q2_pitch: float = 0.0   # rad (elbow relative, same vertical plane)


@dataclass
class FKResult:
    base: Vec3 = (0.0, 0.0, 0.0)
    joint1: Vec3 = (0.0, 0.0, 0.0)  # shoulder location (same as base)
    joint2: Vec3 = (0.0, 0.0, 0.0)  # elbow position
    ee: Vec3 = (0.0, 0.0, 0.0)      # end-effector position


@dataclass
class IKResult:
    reachable: bool
    q: JointAngles
    message: str


def _clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


class RobotArm:
    def __init__(self, link1: LinkParams, link2: LinkParams) -> None:
        self._link1 = link1
        self._link2 = link2
        self._link1.recompute_inertia()
        self._link2.recompute_inertia()

    @property
    def link1(self) -> LinkParams:
        return self._link1

    @property
    def link2(self) -> LinkParams:
        return self._link2

    @property
    def l1(self) -> float:
        return self._link1.length_m

    @property
    def l2(self) -> float:
        return self._link2.length_m

    @property
    def max_reach(self) -> float:
        return self._link1.length_m + self._link2.length_m

    @property
    def min_reach(self) -> float:
        return fabs(self._link1.length_m - self._link2.length_m)

    def solve_ik(self, target: Vec3, elbow_up: bool = False) -> IKResult:
        x, y, z = target

        if z < 0.0:
            return IKResult(False, JointAngles(), "Invalid target: z must be >= 0")

        d = sqrt(x * x + y * y + z * z)
        rmin = self.min_reach
        rmax = self.max_reach
        if d < rmin - 1e-9 or d > rmax + 1e-9:
            return IKResult(
                False,
                JointAngles(),
                f"Target radius |p|={d:.6g} is outside [{rmin:.6g}, {rmax:.6g}]",
            )

        # Base yaw in XY
        q0 = 0.0
        if abs(x) > 1e-12 or abs(y) > 1e-12:
            q0 = atan2(y, x)

        # Reduce to planar IK in (r,z)
        r = sqrt(x * x + y * y)
        L1 = self._link1.length_m
        L2 = self._link2.length_m

        c2 = (r * r + z * z - L1 * L1 - L2 * L2) / (2.0 * L1 * L2)
        c2 = _clamp(c2, -1.0, 1.0)

        q2 = acos(c2)
        if not elbow_up:
            q2 = -q2

        s2 = sin(q2)
        k1 = L1 + L2 * cos(q2)
        k2 = L2 * s2

        q1 = atan2(z, r) - atan2(k2, k1)

        return IKResult(True, JointAngles(q0, q1, q2), "OK")

    def forward_kinematics(self, q: JointAngles) -> FKResult:
        L1 = self._link1.length_m
        L2 = self._link2.length_m

        cy = cos(q.q0_yaw)
        sy = sin(q.q0_yaw)

        # Radial unit in XY and +Z unit
        u = (cy, sy, 0.0)
        k = (0.0, 0.0, 1.0)

        # Elbow
        p1 = (
            u[0] * (L1 * cos(q.q1_pitch)) + k[0] * (L1 * sin(q.q1_pitch)),
            u[1] * (L1 * cos(q.q1_pitch)) + k[1] * (L1 * sin(q.q1_pitch)),
            u[2] * (L1 * cos(q.q1_pitch)) + k[2] * (L1 * sin(q.q1_pitch)),
        )

        # End effector
        a = q.q1_pitch + q.q2_pitch
        p2 = (
            p1[0] + u[0] * (L2 * cos(a)) + k[0] * (L2 * sin(a)),
            p1[1] + u[1] * (L2 * cos(a)) + k[1] * (L2 * sin(a)),
            p1[2] + u[2] * (L2 * cos(a)) + k[2] * (L2 * sin(a)),
        )

        return FKResult(base=(0.0, 0.0, 0.0), joint1=(0.0, 0.0, 0.0), joint2=p1, ee=p2)
