package sim

import "manipulator3d/src/robot"

type LinearTrajectory struct {
	a        robot.Vec3
	b        robot.Vec3
	duration float32
	t        float32
	alpha    float32
	finished bool
}

func NewLinearTrajectory() *LinearTrajectory {
	return &LinearTrajectory{
		a:        robot.Vec3{},
		b:        robot.Vec3{},
		duration: 1.0,
		t:        0,
		alpha:    0,
		finished: true,
	}
}

func (tr *LinearTrajectory) Reset(from, to robot.Vec3, durationSec float32) {
	tr.a = from
	tr.b = to

	if durationSec < 1e-6 {
		durationSec = 1e-6
	}
	tr.duration = durationSec
	tr.t = 0
	tr.alpha = 0
	tr.finished = false
}

func (tr *LinearTrajectory) Update(dt float32) {
	if tr.finished {
		return
	}
	tr.t += dt
	tr.alpha = clamp01(tr.t / tr.duration)
	if tr.alpha >= 1.0 {
		tr.finished = true
	}
}

func (tr *LinearTrajectory) Position() robot.Vec3 {
	return robot.Vec3{
		X: tr.a.X + (tr.b.X-tr.a.X)*tr.alpha,
		Y: tr.a.Y + (tr.b.Y-tr.a.Y)*tr.alpha,
		Z: tr.a.Z + (tr.b.Z-tr.a.Z)*tr.alpha,
	}
}

func (tr *LinearTrajectory) Finished() bool { return tr.finished }
func (tr *LinearTrajectory) Alpha() float32 { return tr.alpha }

func clamp01(x float32) float32 {
	if x < 0 {
		return 0
	}
	if x > 1 {
		return 1
	}
	return x
}
