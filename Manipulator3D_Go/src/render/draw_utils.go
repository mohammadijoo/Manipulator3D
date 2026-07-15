package render

import (
	"math"

	rl "github.com/gen2brain/raylib-go/raylib"
)

func DrawTextBold(font rl.Font, text string, x, y int32, fontSize float32, color rl.Color) {
	pos := rl.NewVector2(float32(x), float32(y))
	rl.DrawTextEx(font, text, pos, fontSize, 1, color)
	rl.DrawTextEx(font, text, rl.NewVector2(float32(x+1), float32(y)), fontSize, 1, color)
	rl.DrawTextEx(font, text, rl.NewVector2(float32(x), float32(y+1)), fontSize, 1, color)
	rl.DrawTextEx(font, text, rl.NewVector2(float32(x+1), float32(y+1)), fontSize, 1, color)
}

func DrawTextSmall(font rl.Font, text string, x, y int32, fontSize float32, color rl.Color) {
	pos := rl.NewVector2(float32(x), float32(y))
	rl.DrawTextEx(font, text, pos, fontSize, 1, color)
}

func DrawRobotBasePedestal(origin rl.Vector3) {
	a := rl.NewVector3(origin.X, origin.Y, -0.25)
	b := rl.NewVector3(origin.X, origin.Y, 0.00)
	rl.DrawCylinderEx(a, b, 0.55, 0.55, 24, rl.NewColor(70, 70, 75, 255))

	c := rl.NewVector3(origin.X, origin.Y, 0.00)
	d := rl.NewVector3(origin.X, origin.Y, 0.35)
	rl.DrawCylinderEx(c, d, 0.38, 0.34, 24, rl.NewColor(95, 95, 100, 255))

	rl.DrawCylinderEx(
		rl.NewVector3(origin.X, origin.Y, 0.00),
		rl.NewVector3(origin.X, origin.Y, 0.06),
		0.48, 0.48, 24,
		rl.NewColor(110, 110, 115, 255),
	)
}

func DrawRobotJointHousing(center rl.Vector3, radius float32) {
	rl.DrawSphere(center, radius, rl.NewColor(120, 120, 125, 255))
	rl.DrawSphereWires(center, radius, 12, 12, rl.NewColor(200, 200, 200, 60))
	rl.DrawCylinderEx(
		rl.NewVector3(center.X, center.Y, center.Z-0.10),
		rl.NewVector3(center.X, center.Y, center.Z+0.10),
		radius*0.55, radius*0.55, 18,
		rl.NewColor(85, 85, 90, 255),
	)
}

func DrawTaperedLink(a, b rl.Vector3, rA, rB float32, color rl.Color) {
	rl.DrawCylinderEx(a, b, rA, rB, 20, color)
	rl.DrawSphere(a, rA*0.95, rl.NewColor(140, 140, 145, 255))
	rl.DrawSphere(b, rB*0.95, rl.NewColor(140, 140, 145, 255))
}

// DrawSuctionTool draws a simple suction-cup tool oriented along approachDir.
func DrawSuctionTool(ee rl.Vector3, approachDir rl.Vector3) {
	// Normalize direction defensively
	alen := float32(math.Sqrt(float64(approachDir.X*approachDir.X + approachDir.Y*approachDir.Y + approachDir.Z*approachDir.Z)))
	dir := approachDir
	if alen > 1e-6 {
		inv := 1.0 / alen
		dir = rl.NewVector3(approachDir.X*inv, approachDir.Y*inv, approachDir.Z*inv)
	} else {
		dir = rl.NewVector3(1, 0, 0)
	}

	tip := rl.NewVector3(ee.X+dir.X*0.28, ee.Y+dir.Y*0.28, ee.Z+dir.Z*0.28)
	rl.DrawCylinderEx(ee, tip, 0.06, 0.05, 18, rl.NewColor(40, 40, 45, 255))

	cupA := tip
	cupB := rl.NewVector3(tip.X+dir.X*0.06, tip.Y+dir.Y*0.06, tip.Z+dir.Z*0.06)
	rl.DrawCylinderEx(cupA, cupB, 0.11, 0.11, 24, rl.NewColor(25, 25, 28, 255))

	rl.DrawSphere(tip, 0.035, rl.NewColor(80, 80, 85, 255))
}
