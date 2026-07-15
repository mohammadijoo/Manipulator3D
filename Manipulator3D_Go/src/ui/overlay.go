package ui

import (
	"fmt"
	"image/color"
	"math"
	"strconv"
	"strings"

	"manipulator3d/src/render"
	"manipulator3d/src/robot"

	rl "github.com/gen2brain/raylib-go/raylib"
)

// drawRoundedBorder draws a rounded-rectangle outline with an approximate pixel thickness.
//
// raylib-go's DrawRectangleRoundedLines does not accept a thickness argument.
// This helper emulates thickness by drawing multiple expanded outlines.
func drawRoundedBorder(r rl.Rectangle, roundness float32, segments int32, thickness int32, c color.RGBA) {
	if thickness <= 1 {
		rl.DrawRectangleRoundedLines(r, roundness, segments, c)
		return
	}
	for i := int32(0); i < thickness; i++ {
		f := float32(i)
		rr := rl.NewRectangle(r.X-f, r.Y-f, r.Width+2*f, r.Height+2*f)
		rl.DrawRectangleRoundedLines(rr, roundness, segments, c)
	}
}

type OverlayStatus struct {
	StartReachable bool
	EndReachable   bool
	ErrorText      string
	PhaseText      string
}

type OverlayAction struct {
	Paused   bool
	ApplyNew bool
	NewStart robot.Vec3
	NewGoal  robot.Vec3
}

type textField struct {
	Label       string
	Text        string
	Rect        rl.Rectangle
	Active      bool
	Invalid     bool
	JustFocused bool
}

type vec3Fields struct {
	Label string
	X     textField
	Y     textField
	Z     textField
}

type Panel struct {
	start vec3Fields
	goal  vec3Fields

	inputError string
	blinkT     float32
}

func NewPanel(start, goal robot.Vec3) *Panel {
	p := &Panel{}
	p.start = makeVec3Fields("START", start)
	p.goal = makeVec3Fields("GOAL", goal)
	return p
}

// Candidate returns the current UI (possibly edited) values without applying them to the simulation.
func (p *Panel) Candidate() (robot.Vec3, robot.Vec3, bool, bool) {
	s, okS := p.start.parse()
	g, okG := p.goal.parse()
	return s, g, okS, okG
}

func (p *Panel) Draw(font rl.Font, arm *robot.RobotArm, status OverlayStatus, paused bool, screenW, screenH int32) OverlayAction {
	_ = screenW
	_ = screenH

	// Blink timer for caret
	p.blinkT += rl.GetFrameTime()
	if p.blinkT > 10.0 {
		p.blinkT = 0
	}

	// Panel frame
	const pad int32 = 12
	x0 := int32(14)
	y0 := int32(14)
	w := int32(360)
	h := int32(560)

	rl.DrawRectangle(x0, y0, w, h, rl.NewColor(18, 18, 18, 230))
	rl.DrawRectangleLines(x0, y0, w, h, rl.NewColor(200, 200, 200, 255))

	y := y0 + pad

	// Title
	render.DrawTextBold(font, "3-DOF 2-Link Arm", x0+pad, y, 24, rl.RayWhite)
	y += 30

	// Phase line
	if status.PhaseText != "" {
		render.DrawTextSmall(font, status.PhaseText, x0+pad, y, 18, rl.SkyBlue)
		y += 24
	}

	// Pause/Play button
	btn := rl.NewRectangle(float32(x0+pad), float32(y), float32(w-2*pad), 36)
	btnBg := rl.NewColor(120, 60, 60, 220)
	if paused {
		btnBg = rl.NewColor(60, 120, 60, 220)
	}
	rl.DrawRectangleRounded(btn, 0.18, 8, btnBg)
	drawRoundedBorder(btn, 0.18, 8, 2, rl.NewColor(230, 230, 230, 255))

	label := "PAUSE"
	if paused {
		label = "PLAY"
	}
	render.DrawTextBold(font, label, int32(btn.X)+12, int32(btn.Y)+7, 22, rl.RayWhite)

	mp := rl.GetMousePosition()
	if rl.IsMouseButtonPressed(rl.MouseButtonLeft) && pointInRect(mp, btn) {
		if paused {
			// Attempt to apply new points when switching to play
			s, g, okS, okG := p.Candidate()
			if !okS || !okG {
				p.inputError = "Invalid input: enter numeric x,y,z (z >= 0)."
				paused = true
			} else {
				p.inputError = ""
				// Clear focus to avoid typing while running
				p.clearFocus()
				return OverlayAction{Paused: false, ApplyNew: true, NewStart: s, NewGoal: g}
			}
		} else {
			paused = true
		}
	}

	y += 46

	// Inputs are editable only while paused
	editable := paused

	// -------------------------
	// Inputs FIRST (top section)
	// -------------------------
	render.DrawTextSmall(font, "Edit START/GOAL while PAUSED. Press PLAY to apply.", x0+pad, y, 16, rl.NewColor(200, 200, 200, 200))
	y += 22

	y = p.drawVec3Input(font, &p.start, x0+pad, y, editable, status.StartReachable)
	y += 10
	y = p.drawVec3Input(font, &p.goal, x0+pad, y, editable, status.EndReachable)
	y += 16

	// Focus handling must be done ONCE per frame (otherwise later widgets clear earlier focus).
	if editable && rl.IsMouseButtonPressed(rl.MouseButtonLeft) && !pointInRect(mp, btn) {
		if f := p.hitTestField(mp); f != nil {
			p.clearFocus()
			f.Active = true
			f.JustFocused = true
		} else {
			// click outside all fields -> clear focus
			p.clearFocus()
		}
	}

	// Divider
	rl.DrawLine(x0+pad, y, x0+w-pad, y, rl.NewColor(255, 255, 255, 35))
	y += 14

	// -------------------------
	// Parameters list (middle)
	// -------------------------
	L1 := arm.Link1()
	L2 := arm.Link2()

	render.DrawTextSmall(font, fmt.Sprintf("Link1 length: %.3fm", L1.LengthM), x0+pad, y, 18, rl.RayWhite)
	y += 20
	render.DrawTextSmall(font, fmt.Sprintf("Link1 mass  : %.3fkg", L1.MassKg), x0+pad, y, 18, rl.RayWhite)
	y += 20
	render.DrawTextSmall(font, fmt.Sprintf("Link1 inertia (joint): %.5f", L1.InertiaJoint), x0+pad, y, 18, rl.RayWhite)
	y += 26

	render.DrawTextSmall(font, fmt.Sprintf("Link2 length: %.3fm", L2.LengthM), x0+pad, y, 18, rl.RayWhite)
	y += 20
	render.DrawTextSmall(font, fmt.Sprintf("Link2 mass  : %.3fkg", L2.MassKg), x0+pad, y, 18, rl.RayWhite)
	y += 20
	render.DrawTextSmall(font, fmt.Sprintf("Link2 inertia (joint): %.5f", L2.InertiaJoint), x0+pad, y, 18, rl.RayWhite)
	y += 24

	rmin := arm.MinReach()
	rmax := arm.MaxReach()
	render.DrawTextSmall(font, fmt.Sprintf("Workspace |p|: [%.2f, %.2f]m", rmin, rmax), x0+pad, y, 18, rl.NewColor(140, 200, 255, 255))
	y += 18

	// -----------------------------------------
	// Logs (below parameters list, as requested)
	// -----------------------------------------
	candStart, candGoal, okS, okG := p.Candidate()

	dsText := "(invalid)"
	dgText := "(invalid)"
	if okS {
		dsText = fmt.Sprintf("%.3f", norm3(candStart))
	}
	if okG {
		dgText = fmt.Sprintf("%.3f", norm3(candGoal))
	}

	// START log
	startCol := rl.Green
	if !status.StartReachable || !okS {
		startCol = rl.Orange
	}
	render.DrawTextSmall(font, "Start |p|="+dsText, x0+pad, y, 18, startCol)
	y += 20
	if okS {
		render.DrawTextSmall(font, fmt.Sprintf("Start: (%.2f, %.2f, %.2f)", candStart.X, candStart.Y, candStart.Z), x0+pad, y, 18, startCol)
	} else {
		render.DrawTextSmall(font, "Start: (invalid)", x0+pad, y, 18, startCol)
	}
	y += 22

	// GOAL log
	goalCol := rl.Green
	if !status.EndReachable || !okG {
		goalCol = rl.Orange
	}
	render.DrawTextSmall(font, "Goal  |p|="+dgText, x0+pad, y, 18, goalCol)
	y += 20
	if okG {
		render.DrawTextSmall(font, fmt.Sprintf("Goal : (%.2f, %.2f, %.2f)", candGoal.X, candGoal.Y, candGoal.Z), x0+pad, y, 18, goalCol)
	} else {
		render.DrawTextSmall(font, "Goal : (invalid)", x0+pad, y, 18, goalCol)
	}
	y += 22

	// Reachability warnings
	if !status.StartReachable || !status.EndReachable {
		render.DrawTextBold(font, "OUT OF REACH!", x0+pad, y, 22, rl.Red)
		y += 24
		render.DrawTextSmall(font, "Choose points inside workspace.", x0+pad, y, 18, rl.Red)
		y += 20
	}

	// Error line (runtime or input)
	if status.ErrorText != "" {
		render.DrawTextBold(font, status.ErrorText, x0+pad, y, 20, rl.Red)
		y += 22
	}
	if p.inputError != "" {
		render.DrawTextSmall(font, p.inputError, x0+pad, y, 18, rl.Orange)
		y += 18
	}

	// Handle text editing
	if editable {
		p.handleTextInput()
	} else {
		p.clearFocus()
	}

	return OverlayAction{Paused: paused}
}

func (p *Panel) drawVec3Input(font rl.Font, vf *vec3Fields, x, y int32, editable bool, reachable bool) int32 {
	// Label
	col := rl.Green
	if !reachable {
		col = rl.Orange
	}
	render.DrawTextBold(font, vf.Label, x, y, 20, col)
	y += 26

	// Boxes
	boxH := float32(30)
	gap := float32(8)
	boxW := float32(104)

	baseY := float32(y)
	padTextX := float32(10)
	padTextY := float32(6)

	fields := []*textField{&vf.X, &vf.Y, &vf.Z}
	for i := range fields {
		r := rl.NewRectangle(float32(x)+float32(i)*(boxW+gap), baseY, boxW, boxH)
		fields[i].Rect = r

		bg := rl.NewColor(30, 30, 32, 235)
		if !editable {
			bg = rl.NewColor(26, 26, 28, 225)
		}
		rl.DrawRectangleRounded(r, 0.18, 8, bg)

		border := rl.NewColor(230, 230, 230, 120)
		if fields[i].Active && editable {
			border = rl.NewColor(240, 240, 240, 230)
		}
		if fields[i].Invalid {
			border = rl.NewColor(255, 120, 90, 230)
		}
		drawRoundedBorder(r, 0.18, 8, 2, border)

		// Text
		txt := fields[i].Text
		render.DrawTextSmall(font, txt, int32(r.X+padTextX), int32(r.Y+padTextY), 18, rl.RayWhite)

		// Caret (only when active + editable)
		if editable && fields[i].Active && caretOn(p.blinkT) {
			measure := rl.MeasureTextEx(font, txt, 18, 1.0)
			cx := r.X + padTextX + measure.X + 2
			cy := r.Y + 6
			rl.DrawRectangle(int32(cx), int32(cy), 2, 18, rl.NewColor(255, 255, 255, 220))
		}
	}

	// Field captions
	y2 := y + 34
	render.DrawTextSmall(font, "x", x, y2, 16, rl.NewColor(200, 200, 200, 200))
	render.DrawTextSmall(font, "y", x+int32(boxW+gap), y2, 16, rl.NewColor(200, 200, 200, 200))
	render.DrawTextSmall(font, "z", x+int32(2*(boxW+gap)), y2, 16, rl.NewColor(200, 200, 200, 200))

	return y + 52
}

// hitTestField returns the field under the mouse (if any).
func (p *Panel) hitTestField(mp rl.Vector2) *textField {
	// START fields
	if pointInRect(mp, p.start.X.Rect) {
		return &p.start.X
	}
	if pointInRect(mp, p.start.Y.Rect) {
		return &p.start.Y
	}
	if pointInRect(mp, p.start.Z.Rect) {
		return &p.start.Z
	}
	// GOAL fields
	if pointInRect(mp, p.goal.X.Rect) {
		return &p.goal.X
	}
	if pointInRect(mp, p.goal.Y.Rect) {
		return &p.goal.Y
	}
	if pointInRect(mp, p.goal.Z.Rect) {
		return &p.goal.Z
	}
	return nil
}

func (p *Panel) handleTextInput() {
	if rl.IsKeyPressed(rl.KeyEscape) {
		p.clearFocus()
		return
	}

	active := p.activeField()
	if active == nil {
		return
	}

	// Delete clears the field
	if rl.IsKeyPressed(rl.KeyDelete) {
		active.Text = ""
		active.JustFocused = false
		active.Invalid = false
		return
	}

	// Backspace
	if rl.IsKeyPressed(rl.KeyBackspace) {
		if active.JustFocused {
			active.Text = ""
			active.JustFocused = false
		} else if len(active.Text) > 0 {
			active.Text = active.Text[:len(active.Text)-1]
		}
	}

	// Typed characters
	for {
		ch := rl.GetCharPressed()
		if ch == 0 {
			break
		}
		r := rune(ch)

		// First keypress after focus replaces existing text
		if active.JustFocused {
			active.Text = ""
			active.JustFocused = false
		}

		if !isNumericRune(r, active.Text) {
			continue
		}
		if len(active.Text) >= 12 {
			continue
		}
		active.Text += string(r)
	}

	// Validate field formatting
	active.Invalid = !looksLikeFloat(active.Text)

	// Extra validation: z must be >= 0
	if active == &p.start.Z || active == &p.goal.Z {
		if z, ok := parseFloat32(active.Text); ok {
			if z < 0 {
				active.Invalid = true
			}
		}
	}
}

func (p *Panel) activeField() *textField {
	if p.start.X.Active {
		return &p.start.X
	}
	if p.start.Y.Active {
		return &p.start.Y
	}
	if p.start.Z.Active {
		return &p.start.Z
	}
	if p.goal.X.Active {
		return &p.goal.X
	}
	if p.goal.Y.Active {
		return &p.goal.Y
	}
	if p.goal.Z.Active {
		return &p.goal.Z
	}
	return nil
}

func (p *Panel) clearFocus() {
	p.start.X.Active, p.start.Y.Active, p.start.Z.Active = false, false, false
	p.goal.X.Active, p.goal.Y.Active, p.goal.Z.Active = false, false, false

	p.start.X.JustFocused, p.start.Y.JustFocused, p.start.Z.JustFocused = false, false, false
	p.goal.X.JustFocused, p.goal.Y.JustFocused, p.goal.Z.JustFocused = false, false, false
}

func makeVec3Fields(label string, v robot.Vec3) vec3Fields {
	return vec3Fields{
		Label: label,
		X:     textField{Label: "x", Text: fmt.Sprintf("%.2f", v.X)},
		Y:     textField{Label: "y", Text: fmt.Sprintf("%.2f", v.Y)},
		Z:     textField{Label: "z", Text: fmt.Sprintf("%.2f", v.Z)},
	}
}

func (vf *vec3Fields) parse() (robot.Vec3, bool) {
	x, okX := parseFloat32(vf.X.Text)
	y, okY := parseFloat32(vf.Y.Text)
	z, okZ := parseFloat32(vf.Z.Text)

	vf.X.Invalid = !okX
	vf.Y.Invalid = !okY
	vf.Z.Invalid = !okZ

	if !okX || !okY || !okZ {
		return robot.Vec3{}, false
	}
	if z < 0 {
		vf.Z.Invalid = true
		return robot.Vec3{}, false
	}
	return robot.Vec3{X: x, Y: y, Z: z}, true
}

func parseFloat32(s string) (float32, bool) {
	s = strings.TrimSpace(s)
	if s == "" || s == "-" || s == "." || s == "-." {
		return 0, false
	}
	f, err := strconv.ParseFloat(s, 32)
	if err != nil {
		return 0, false
	}
	return float32(f), true
}

func pointInRect(p rl.Vector2, r rl.Rectangle) bool {
	return p.X >= r.X && p.X <= r.X+r.Width && p.Y >= r.Y && p.Y <= r.Y+r.Height
}

func looksLikeFloat(s string) bool {
	s = strings.TrimSpace(s)
	if s == "" || s == "-" || s == "." || s == "-." {
		return true
	}
	_, err := strconv.ParseFloat(s, 32)
	return err == nil
}

func isNumericRune(r rune, current string) bool {
	if r >= '0' && r <= '9' {
		return true
	}
	if r == '-' {
		return len(current) == 0
	}
	if r == '.' {
		return !strings.ContainsRune(current, '.')
	}
	return false
}

func caretOn(t float32) bool {
	return int(math.Floor(float64(t*2.0)))%2 == 0
}

func norm3(v robot.Vec3) float32 {
	return float32(math.Sqrt(float64(v.X*v.X + v.Y*v.Y + v.Z*v.Z)))
}
