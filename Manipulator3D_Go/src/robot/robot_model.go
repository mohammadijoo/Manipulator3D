package robot

import (
	"math"
)

type LinkParams struct {
	LengthM float32
	MassKg  float32

	// Uniform rod approximations:
	// I_cm   = (1/12) m L^2
	// I_joint= (1/3)  m L^2
	InertiaCM    float32
	InertiaJoint float32
}

func (lp *LinkParams) RecomputeInertia() {
	L2 := lp.LengthM * lp.LengthM
	lp.InertiaCM = (1.0 / 12.0) * lp.MassKg * L2
	lp.InertiaJoint = (1.0 / 3.0) * lp.MassKg * L2
}

type JointAngles struct {
	Q0Yaw   float32 // rad
	Q1Pitch float32 // rad (relative to XY plane)
	Q2Pitch float32 // rad (elbow relative, same plane)
}

type FKResult struct {
	Base   Vec3
	Joint1 Vec3
	Joint2 Vec3
	EE     Vec3
}

type IKResult struct {
	Reachable bool
	Q         JointAngles
	Message   string
}

type RobotArm struct {
	link1 LinkParams
	link2 LinkParams
}

func NewRobotArm(l1, l2 LinkParams) *RobotArm {
	l1.RecomputeInertia()
	l2.RecomputeInertia()
	return &RobotArm{link1: l1, link2: l2}
}

func (a *RobotArm) L1() float32 { return a.link1.LengthM }
func (a *RobotArm) L2() float32 { return a.link2.LengthM }

func (a *RobotArm) MaxReach() float32 { return a.link1.LengthM + a.link2.LengthM }
func (a *RobotArm) MinReach() float32 { return float32(math.Abs(float64(a.link1.LengthM - a.link2.LengthM))) }

func (a *RobotArm) Link1() LinkParams { return a.link1 }
func (a *RobotArm) Link2() LinkParams { return a.link2 }

// SolveIK computes a closed-form IK solution for a 3D target.
// elbowUp selects the elbow-up vs elbow-down branch.
func (a *RobotArm) SolveIK(target Vec3, elbowUp bool) IKResult {
	out := IKResult{Reachable: false}

	if target.Z < 0 {
		out.Message = "Invalid target: z must be >= 0"
		return out
	}

	x := float64(target.X)
	y := float64(target.Y)
	z := float64(target.Z)

	d := math.Sqrt(x*x + y*y + z*z)
	rmin := float64(a.MinReach())
	rmax := float64(a.MaxReach())

	const eps = 1e-9
	if d < rmin-eps || d > rmax+eps {
		out.Message = "Target radius |p| is outside workspace"
		return out
	}

	// Base yaw
	q0 := 0.0
	if math.Abs(x) > 1e-12 || math.Abs(y) > 1e-12 {
		q0 = math.Atan2(y, x)
	}

	// Reduce to planar IK in (r,z)
	r := math.Sqrt(x*x + y*y)
	L1 := float64(a.link1.LengthM)
	L2 := float64(a.link2.LengthM)

	c2 := (r*r + z*z - L1*L1 - L2*L2) / (2.0 * L1 * L2)
	c2 = clampd(c2, -1.0, 1.0)

	q2 := math.Acos(c2)
	if !elbowUp {
		q2 = -q2
	}

	s2 := math.Sin(q2)
	k1 := L1 + L2*math.Cos(q2)
	k2 := L2 * s2

	q1 := math.Atan2(z, r) - math.Atan2(k2, k1)

	out.Reachable = true
	out.Q = JointAngles{
		Q0Yaw:   float32(q0),
		Q1Pitch: float32(q1),
		Q2Pitch: float32(q2),
	}
	out.Message = "OK"
	return out
}

// ForwardKinematics returns the base/joint/end-effector positions for the provided angles.
func (a *RobotArm) ForwardKinematics(q JointAngles) FKResult {
	fk := FKResult{}
	fk.Base = Vec3{0, 0, 0}
	fk.Joint1 = fk.Base

	L1 := float64(a.link1.LengthM)
	L2 := float64(a.link2.LengthM)

	cy := math.Cos(float64(q.Q0Yaw))
	sy := math.Sin(float64(q.Q0Yaw))

	u := Vec3{float32(cy), float32(sy), 0}
	k := Vec3{0, 0, 1}

	p1 := add(
		scale(u, float32(L1*math.Cos(float64(q.Q1Pitch)))),
		scale(k, float32(L1*math.Sin(float64(q.Q1Pitch)))),
	)

	a2 := float64(q.Q1Pitch + q.Q2Pitch)
	p2 := add(
		p1,
		add(
			scale(u, float32(L2*math.Cos(a2))),
			scale(k, float32(L2*math.Sin(a2))),
		),
	)

	fk.Joint2 = p1
	fk.EE = p2
	return fk
}

// --------------------
// Minimal Vec3 helpers
// --------------------

type Vec3 struct {
	X, Y, Z float32
}

func add(a, b Vec3) Vec3  { return Vec3{a.X + b.X, a.Y + b.Y, a.Z + b.Z} }
func sub(a, b Vec3) Vec3  { return Vec3{a.X - b.X, a.Y - b.Y, a.Z - b.Z} }
func scale(v Vec3, s float32) Vec3 {
	return Vec3{v.X * s, v.Y * s, v.Z * s}
}
func len3(v Vec3) float32 {
	return float32(math.Sqrt(float64(v.X*v.X + v.Y*v.Y + v.Z*v.Z)))
}
func normalize(v Vec3) Vec3 {
	L := len3(v)
	if L < 1e-8 {
		return Vec3{1, 0, 0}
	}
	inv := 1.0 / L
	return scale(v, inv)
}
func clampd(v, lo, hi float64) float64 {
	if v < lo {
		return lo
	}
	if v > hi {
		return hi
	}
	return v
}
