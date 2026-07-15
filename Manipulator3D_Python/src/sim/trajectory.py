from __future__ import annotations

from dataclasses import dataclass
from typing import Tuple


Vec3 = Tuple[float, float, float]


def _clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


@dataclass
class LinearTrajectory:
    a: Vec3 = (0.0, 0.0, 0.0)
    b: Vec3 = (0.0, 0.0, 0.0)
    duration: float = 1.0
    t: float = 0.0
    alpha: float = 0.0
    finished: bool = True

    def reset(self, from_pos: Vec3, to_pos: Vec3, duration_sec: float) -> None:
        self.a = from_pos
        self.b = to_pos
        self.duration = duration_sec if duration_sec > 1e-6 else 1e-6
        self.t = 0.0
        self.alpha = 0.0
        self.finished = False

    def update(self, dt: float) -> None:
        if self.finished:
            return
        self.t += dt
        self.alpha = _clamp(self.t / self.duration, 0.0, 1.0)
        if self.alpha >= 1.0:
            self.finished = True

    def position(self) -> Vec3:
        ax, ay, az = self.a
        bx, by, bz = self.b
        a = self.alpha
        return (ax + (bx - ax) * a, ay + (by - ay) * a, az + (bz - az) * a)
