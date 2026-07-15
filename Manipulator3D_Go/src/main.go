package main

import (
	"math"
	"os"

	"manipulator3d/src/render"
	"manipulator3d/src/robot"
	"manipulator3d/src/sim"
	"manipulator3d/src/ui"

	rl "github.com/gen2brain/raylib-go/raylib"
)

type Phase int

const (
	PhaseMoveHomeToStart Phase = iota
	PhasePickAtStart
	PhaseMoveStartToGoal
	PhasePlaceAtGoal
	PhaseReturnGoalToHome
	PhaseWaitAtHomeReset
	PhaseError
)

type BallState int

const (
	BallAtStart BallState = iota
	BallAttached
	BallAtGoal
)

func fileExists(path string) bool {
	_, err := os.Stat(path)
	return err == nil
}

func loadBestUIFont(px int32) rl.Font {
	candidates := []string{
		"resources/fonts/Inter-Regular.ttf",
		"../resources/fonts/Inter-Regular.ttf",
		"C:/Windows/Fonts/segoeui.ttf",
		"C:/Windows/Fonts/arial.ttf",
	}

	for _, path := range candidates {
		if !fileExists(path) {
			continue
		}
		f := rl.LoadFontEx(path, px, nil)
		if f.Texture.ID != 0 {
			rl.SetTextureFilter(f.Texture, rl.FilterBilinear)
			return f
		}
	}

	def := rl.GetFontDefault()
	rl.SetTextureFilter(def.Texture, rl.FilterBilinear)
	return def
}

func updateZoom(cam *rl.Camera3D) {
	wheel := rl.GetMouseWheelMove()
	if wheel == 0 {
		return
	}

	v := rl.Vector3Subtract(cam.Position, cam.Target)
	dist := rl.Vector3Length(v)
	if dist < 0.001 {
		dist = 0.001
	}

	scale := 1.0 - float32(wheel)*0.10
	if scale < 0.70 {
		scale = 0.70
	} else if scale > 1.30 {
		scale = 1.30
	}

	dist *= scale
	if dist < 1.0 {
		dist = 1.0
	} else if dist > 200.0 {
		dist = 200.0
	}

	dir := rl.Vector3Normalize(v)
	cam.Position = rl.Vector3Add(cam.Target, rl.Vector3Scale(dir, dist))
}

func vec3ToRL(v robot.Vec3) rl.Vector3 {
	return rl.NewVector3(v.X, v.Y, v.Z)
}

func norm(v robot.Vec3) float32 {
	return float32(math.Sqrt(float64(v.X*v.X + v.Y*v.Y + v.Z*v.Z)))
}

func main() {
	rl.SetTraceLogLevel(rl.LogError)
	rl.SetConfigFlags(rl.FlagWindowResizable | rl.FlagMsaa4xHint)

	// --- Arm parameters ---
	link1 := robot.LinkParams{LengthM: 3.0, MassKg: 2.0}
	link1.RecomputeInertia()
	link2 := robot.LinkParams{LengthM: 2.6, MassKg: 1.6}
	link2.RecomputeInertia()

	arm := robot.NewRobotArm(link1, link2) // *robot.RobotArm

	// Defaults
	start := robot.Vec3{X: 1, Y: 2, Z: 1}
	goal := robot.Vec3{X: 2, Y: 3, Z: 2}

	// Fixed HOME EE position
	homeEE := robot.Vec3{X: 2, Y: 2, Z: 2}

	rl.InitWindow(1280, 720, "Manipulator3D - 3DOF IK Pick&Place (Go)")
	rl.SetWindowMinSize(960, 540)
	rl.SetTargetFPS(60)

	uiFont := loadBestUIFont(22)

	// Camera framing
	reach := arm.MaxReach()
	var cam rl.Camera3D
	cam.Target = rl.NewVector3(0, 0, 0.35*reach)
	cam.Up = rl.NewVector3(0, 0, 1)
	cam.Fovy = 52
	cam.Projection = rl.CameraPerspective
	cam.Position = rl.NewVector3(1.10*reach, -1.15*reach, 0.85*reach)

	// UI panel (holds editable text)
	panel := ui.NewPanel(start, goal)

	// Start paused: user edits fields then hits PLAY
	paused := true

	// Timing: manipulator speed +50% (movement durations / 1.5)
	speedFactor := float32(1.5)

	moveHomeToStart := float32(2.2) / speedFactor
	moveStartToGoal := float32(2.6) / speedFactor
	returnToHome := float32(2.0) / speedFactor

	// Keep pick/place/wait unchanged
	pickDuration := float32(0.45)
	placeDuration := float32(0.35)
	resetWaitTotal := float32(1.5)

	timer := float32(0)

	// Ball size
	ballRadius := float32(0.03) * reach
	if ballRadius < 0.06 {
		ballRadius = 0.06
	} else if ballRadius > 0.16 {
		ballRadius = 0.16
	}

	ballState := BallAtStart
	ballPos := start

	targetEE := homeEE
	qcmd := robot.JointAngles{}

	traj := sim.NewLinearTrajectory()

	phase := PhaseWaitAtHomeReset
	runtimeErr := ""

	// Initialize at HOME pose if reachable
	if ikHome := arm.SolveIK(homeEE, false); ikHome.Reachable {
		qcmd = ikHome.Q
		targetEE = homeEE
		ballState = BallAtStart
		ballPos = start
		runtimeErr = ""
	} else {
		phase = PhaseError
		runtimeErr = "ERROR: HOME is not reachable."
	}

	for !rl.WindowShouldClose() {
		if rl.IsKeyPressed(rl.KeyF11) {
			rl.ToggleFullscreen()
		}

		updateZoom(&cam)

		dt := rl.GetFrameTime()
		if paused {
			dt = 0
		}

		// Live reachability uses current UI candidate values
		candStart, candGoal, okS, okG := panel.Candidate()

		startReachable := false
		endReachable := false
		startMsg := ""
		goalMsg := ""

		if okS {
			ik := arm.SolveIK(candStart, false)
			startReachable = ik.Reachable
			if !ik.Reachable {
				startMsg = ik.Message
			}
		}
		if okG {
			ik := arm.SolveIK(candGoal, false)
			endReachable = ik.Reachable
			if !ik.Reachable {
				goalMsg = ik.Message
			}
		}

		// Simulation state machine
		if phase != PhaseError {
			switch phase {
			case PhaseMoveHomeToStart:
				ballState = BallAtStart
				ballPos = start

				traj.Update(dt)
				targetEE = traj.Position()

				if traj.Finished() {
					phase = PhasePickAtStart
					timer = 0
					targetEE = start
				}

			case PhasePickAtStart:
				targetEE = start
				ballState = BallAtStart
				ballPos = start

				timer += dt
				if timer >= pickDuration {
					ballState = BallAttached
					timer = 0
					traj.Reset(start, goal, moveStartToGoal)
					phase = PhaseMoveStartToGoal
				}

			case PhaseMoveStartToGoal:
				traj.Update(dt)
				targetEE = traj.Position()
				ballState = BallAttached

				if traj.Finished() {
					phase = PhasePlaceAtGoal
					timer = 0
					targetEE = goal
				}

			case PhasePlaceAtGoal:
				targetEE = goal

				timer += dt
				if timer >= placeDuration {
					ballState = BallAtGoal
					timer = 0
					traj.Reset(goal, homeEE, returnToHome)
					phase = PhaseReturnGoalToHome
				} else {
					ballState = BallAttached
				}

			case PhaseReturnGoalToHome:
				ballState = BallAtGoal

				traj.Update(dt)
				targetEE = traj.Position()

				timer += dt
				if traj.Finished() {
					phase = PhaseWaitAtHomeReset
				}

			case PhaseWaitAtHomeReset:
				targetEE = homeEE
				timer += dt
				if timer >= resetWaitTotal {
					ballState = BallAtStart
					ballPos = start

					timer = 0
					traj.Reset(homeEE, start, moveHomeToStart)
					phase = PhaseMoveHomeToStart
				} else {
					ballState = BallAtGoal
				}
			}

			ikNow := arm.SolveIK(targetEE, false)
			if !ikNow.Reachable {
				phase = PhaseError
				runtimeErr = ikNow.Message
			} else {
				qcmd = ikNow.Q
			}
		}

		// FK for rendering + ball attachment offset
		fk := arm.ForwardKinematics(qcmd)

		approach := robot.Vec3{
			X: fk.EE.X - fk.Joint2.X,
			Y: fk.EE.Y - fk.Joint2.Y,
			Z: fk.EE.Z - fk.Joint2.Z,
		}
		alen := norm(approach)
		if alen > 1e-6 {
			approach.X /= alen
			approach.Y /= alen
			approach.Z /= alen
		} else {
			approach = robot.Vec3{X: 1, Y: 0, Z: 0}
		}

		if ballState == BallAtStart {
			ballPos = start
		} else if ballState == BallAtGoal {
			ballPos = goal
		} else {
			ballPos = robot.Vec3{
				X: fk.EE.X + approach.X*0.22,
				Y: fk.EE.Y + approach.Y*0.22,
				Z: fk.EE.Z + approach.Z*0.22,
			}
		}

		// Draw
		rl.BeginDrawing()
		rl.ClearBackground(rl.NewColor(10, 12, 16, 255))

		rl.BeginMode3D(cam)

		// Thicker axes using cylinders
		axisLen := float32(3.0)
		axisR := float32(0.03)
		rl.DrawCylinderEx(rl.NewVector3(0, 0, 0), rl.NewVector3(axisLen, 0, 0), axisR, axisR, 12, rl.Red)
		rl.DrawCylinderEx(rl.NewVector3(0, 0, 0), rl.NewVector3(0, axisLen, 0), axisR, axisR, 12, rl.Green)
		rl.DrawCylinderEx(rl.NewVector3(0, 0, 0), rl.NewVector3(0, 0, axisLen), axisR, axisR, 12, rl.Blue)

		render.DrawRobotBasePedestal(rl.NewVector3(0, 0, 0))
		render.DrawRobotJointHousing(vec3ToRL(fk.Base), 0.30)
		render.DrawRobotJointHousing(vec3ToRL(fk.Joint2), 0.24)
		render.DrawRobotJointHousing(vec3ToRL(fk.EE), 0.18)

		render.DrawTaperedLink(vec3ToRL(fk.Base), vec3ToRL(fk.Joint2), 0.14, 0.12, rl.NewColor(185, 185, 190, 255))
		render.DrawTaperedLink(vec3ToRL(fk.Joint2), vec3ToRL(fk.EE), 0.12, 0.10, rl.NewColor(170, 170, 175, 255))

		render.DrawSuctionTool(vec3ToRL(fk.EE), vec3ToRL(approach))

		rl.DrawSphere(vec3ToRL(ballPos), ballRadius, rl.Red)
		rl.DrawSphereWires(vec3ToRL(ballPos), ballRadius*1.02, 10, 10, rl.RayWhite)

		rl.EndMode3D()

		// Phase label
		phaseText := ""
		switch phase {
		case PhaseMoveHomeToStart:
			phaseText = "Phase: HOME -> START"
		case PhasePickAtStart:
			phaseText = "Phase: PICK at START"
		case PhaseMoveStartToGoal:
			phaseText = "Phase: START -> GOAL (ball attached)"
		case PhasePlaceAtGoal:
			phaseText = "Phase: PLACE at GOAL"
		case PhaseReturnGoalToHome:
			phaseText = "Phase: GOAL -> HOME"
		case PhaseWaitAtHomeReset:
			phaseText = "Phase: WAIT then LOOP"
		case PhaseError:
			phaseText = "Phase: ERROR"
		}

		errText := runtimeErr
		if paused {
			if okS && !startReachable && startMsg != "" {
				errText = "START: " + startMsg
			} else if okG && !endReachable && goalMsg != "" {
				errText = "GOAL: " + goalMsg
			}
		}
		if phase == PhaseError && errText == "" {
			errText = "ERROR: invalid or out of reach (z>=0 required)."
		}

		status := ui.OverlayStatus{
			StartReachable: startReachable,
			EndReachable:   endReachable,
			ErrorText:      errText,
			PhaseText:      phaseText,
		}

		// Cast screen sizes to int32 (raylib-go returns int here)
		sw := int32(rl.GetScreenWidth())
		sh := int32(rl.GetScreenHeight())

		action := panel.Draw(uiFont, arm, status, paused, sw, sh)
		paused = action.Paused

		if action.ApplyNew {
			start = action.NewStart
			goal = action.NewGoal

			ikHome := arm.SolveIK(homeEE, false)
			ikStart := arm.SolveIK(start, false)
			ikGoal := arm.SolveIK(goal, false)

			if !ikHome.Reachable || !ikStart.Reachable || !ikGoal.Reachable {
				phase = PhaseError
				runtimeErr = "ERROR: HOME/START/GOAL invalid or out of reach."
				paused = true
			} else {
				qcmd = ikHome.Q
				targetEE = homeEE

				ballState = BallAtStart
				ballPos = start

				timer = 0
				traj.Reset(homeEE, start, moveHomeToStart)
				phase = PhaseMoveHomeToStart
				runtimeErr = ""
			}
		}

		// bottom help text (cast y to int32)
		render.DrawTextSmall(uiFont, "F11: fullscreen   Mouse Wheel: zoom", 12, sh-28, 18, rl.NewColor(200, 200, 200, 220))
		rl.EndDrawing()
	}

	if uiFont.Texture.ID != rl.GetFontDefault().Texture.ID {
		rl.UnloadFont(uiFont)
	}
	rl.CloseWindow()
}
