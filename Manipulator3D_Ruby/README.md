<div style="font-family: system-ui, -apple-system, BlinkMacSystemFont, 'Segoe UI', sans-serif; line-height: 1.6;">

  <h1 align="center" style="margin-bottom: 0.2em;">Manipulator3D — 3-DOF Two-Link Robot Arm (Analytic IK + Pick&amp;Place)</h1>

  <p style="font-size: 0.98rem; max-width: 920px; margin: 0.2rem auto 0;">
    A Ruby simulation and visualization project for a <strong>3-DOF, two-link manipulator</strong> with a <strong>fixed base</strong> in 3D space.
    The arm executes a simple <strong>pick-and-place loop</strong> using an <strong>analytic inverse kinematics</strong> solver and a <strong>linear end-effector trajectory</strong>.
  </p>

  <ul style="max-width: 920px; margin: 0.5rem auto 0.2rem; padding-left: 1.2rem;">
    <li><strong>Joint 0 (base):</strong> revolute yaw about the <strong>+Z axis</strong> at the origin <code>(0,0,0)</code>.</li>
    <li><strong>Joints 1–2 (links):</strong> revolute pitch angles defining motion in the arm’s vertical plane (relative to the <strong>X–Y plane</strong>).</li>
    <li><strong>IK:</strong> reduces the 3D target to a 2D planar problem in <code>(r,z)</code>, solves with the <strong>law of cosines</strong>, and supports elbow-up / elbow-down branches.</li>
    <li><strong>Visualization:</strong> rendered with <strong>raylib</strong> via <strong>raylib-bindings</strong>, including an overlay panel and a suction-tool “ball” pick-and-place animation.</li>
  </ul>

  <p align="center" style="font-size: 1rem; color: #666; margin: 0.35rem auto 0;">
    Language: <strong>Ruby</strong> • Rendering: <strong>raylib</strong> • Binding: <strong>raylib-bindings</strong> • IK: <strong>Closed-form</strong>
  </p>

</div>

---

<!-- ========================================================= -->
<!-- Table of Contents                                        -->
<!-- ========================================================= -->

<ul style="list-style: none; padding-left: 0; font-size: 0.97rem;">
  <li> <a href="#about-this-repository">About this repository</a></li>
  <li> <a href="#repository-structure">Repository structure</a></li>
  <li> <a href="#robotics-kinematics-dynamics-and-ik">Robotics: kinematics, dynamics, and inverse kinematics</a></li>
  <li> <a href="#installing-dependencies">Installing dependencies</a></li>
  <li> <a href="#running-the-simulator">Running the simulator</a></li>
  <li> <a href="#repository-file-guide">Repository file guide (full explanation)</a></li>
  <li> <a href="#simulation-video">Simulation video</a></li>
  <li> <a href="#troubleshooting">Troubleshooting</a></li>
</ul>

---

<a id="about-this-repository"></a>

## About this repository

This project simulates a **3-DOF, two-link manipulator** mounted on a **fixed base** at the origin:

- The base joint rotates around the **Z axis** (yaw).
- The two link joints rotate as **pitch angles** (relative to the X–Y plane) in the arm’s vertical plane.
- The end-effector (EE) tracks a sequence of target points with **analytic inverse kinematics**.
- A small “ball” is used to visualize a **pick-and-place loop**:
  1) HOME → START  
  2) PICK at START (attach ball)  
  3) START → GOAL  
  4) PLACE at GOAL (detach ball)  
  5) GOAL → HOME  
  6) wait briefly and repeat  

**User inputs (via UI overlay):**
- START position `(x, y, z)` and GOAL position `(x, y, z)` are entered in the on-screen panel.
- Press **PAUSE** to edit fields safely.
- Press **PLAY (apply inputs)** to validate and start a new loop with the new START/GOAL.
- Constraint enforced by IK: **z must be ≥ 0**
- Reachability enforced by IK: `|p|` must be within the arm’s reachable shell.

**Font:**
- The UI loads `resources/fonts/Inter-Regular.ttf` when present (recommended).

---

<a id="repository-structure"></a>

## Repository structure

```txt
Manipulator3D_Ruby/
  Gemfile
  Gemfile.lock
  README.md
  resources/
    fonts/
      Inter-Regular.ttf
  src/
    main.rb
    raylib_bootstrap.rb
    util/
      math3d.rb
    robot/
      robot_arm.rb
    sim/
      trajectory.rb
    ui/
      overlay.rb
    render/
      draw_utils.rb
```

---

<a id="robotics-kinematics-dynamics-and-ik"></a>

## Robotics: kinematics, dynamics, and inverse kinematics

### 1) Coordinate frames and joint conventions

- World frame origin is at the center of the base revolute joint: **(0,0,0)**.
- Joint angles:
  - `q0_yaw` : rotation about **+Z** (sets the arm’s radial direction in the X–Y plane)
  - `q1_pitch`: shoulder elevation angle relative to the **X–Y plane**
  - `q2_pitch`: elbow pitch angle (relative bend in the same vertical plane)

We use the radial unit direction in the X–Y plane:

```math
\mathbf{u} =
\begin{bmatrix}
\cos(q_0)\\
\sin(q_0)\\
0
\end{bmatrix},
\quad
\mathbf{k} =
\begin{bmatrix}
0\\
0\\
1
\end{bmatrix}.
```

### 2) Forward kinematics (FK)

Let link lengths be $L_1$ and $L_2$. Then:

- Elbow position:

```math
\mathbf{p}_1 = L_1\cos(q_1)\,\mathbf{u} + L_1\sin(q_1)\,\mathbf{k}
```

- End-effector position:

```math
\mathbf{p}_{ee} = \mathbf{p}_1 + L_2\cos(q_1+q_2)\,\mathbf{u} + L_2\sin(q_1+q_2)\,\mathbf{k}
```

Implemented in:
- `Robot::RobotArm#forward_kinematics`

### 3) Workspace (reachability) check

This manipulator is effectively a 2-link arm in a vertical plane, rotated by yaw. Therefore the reachable radius must satisfy:

```math
r_{\min} = |L_1 - L_2|,\quad r_{\max} = L_1 + L_2
```

```math
\|\mathbf{p}\| \in [r_{\min}, r_{\max}]
```

The implementation enforces:
- `target.z >= 0`
- `|p|` within `[min_reach, max_reach]`

### 4) Analytic inverse kinematics (IK): how it works here

Given target $\mathbf{p} = (x,y,z)$:

**Step A — base yaw**

```math
q_0 = \mathrm{atan2}(y, x)
```

**Step B — reduce to planar IK in $(r,z)$**

```math
r = \sqrt{x^2 + y^2}
```

**Step C — elbow angle from law of cosines**

```math
\cos(q_2) = \frac{r^2 + z^2 - L_1^2 - L_2^2}{2L_1L_2}
```

Clamp to $[-1,1]$ for numerical safety, then:

```math
q_2 = \pm \arccos(\cos(q_2))
```

**Step D — shoulder angle**

Define:

```math
k_1 = L_1 + L_2\cos(q_2),\quad k_2 = L_2\sin(q_2)
```

Then:

```math
q_1 = \mathrm{atan2}(z, r) - \mathrm{atan2}(k_2, k_1)
```

Implemented in:
- `Robot::RobotArm#solve_ik`

### 5) Dynamics (what is implemented vs. what is prepared)

This repository is primarily a **kinematic + trajectory** simulator:

- The EE follows a commanded Cartesian trajectory.
- IK converts EE targets into joint angles.
- FK is used for rendering joint/link positions.

The code also defines **mass and inertia properties** for each link (uniform rod approximations):

- About center of mass:

```math
I_{cm} = \frac{1}{12} mL^2
```

- About the joint at one end:

```math
I_{joint} = \frac{1}{3} mL^2
```

Computed in:
- `Robot::LinkParams#recompute_inertia`

These values are displayed in the overlay panel and can be used as a foundation for extending the simulator to include:
- forward dynamics (torques → accelerations),
- gravity and Coriolis terms,
- joint-space controllers (PD, computed torque, etc.).

---

<a id="installing-dependencies"></a>

## Installing dependencies

### Requirements

- Ruby (recommended: Ruby 3.x)
- Bundler (usually included with Ruby)

This project uses:

- `raylib-bindings` (raylib + helper libraries shipped as platform binaries)
- `ffi` (pulled automatically as a dependency)

### Install

From the repository root:

```bash
bundle install
```

---

<a id="running-the-simulator"></a>

## Running the simulator

From the repository root:

```bash
bundle exec ruby src/main.rb
```

### Controls / interaction

- Mouse wheel: zoom camera
- F11: toggle fullscreen
- Overlay includes:
  - PAUSE / PLAY (apply inputs)
  - START/GOAL editable fields
  - reachability feedback

---

<a id="repository-file-guide"></a>

## Repository file guide (full explanation)

This section explains every important file in the repository and its role.

### `Gemfile`

Declares Ruby dependencies. The core dependency is `raylib-bindings`.

---

### `src/raylib_bootstrap.rb`

Loads the raylib shared libraries shipped by `raylib-bindings` and enables direct calls to raylib functions (e.g., `InitWindow`, `BeginDrawing`, ...).

---

### `src/main.rb`

Application entry point and runtime loop:

- initializes window + camera
- loads UI font (prefers `resources/fonts/Inter-Regular.ttf`)
- runs the pick-and-place finite-state machine:
  - HOME → START → PICK → GOAL → PLACE → HOME → WAIT → LOOP
- generates a linear Cartesian trajectory between targets
- runs IK each frame to get joint angles for the current EE target
- calls FK for rendering joint/link positions
- renders:
  - robot geometry, axes, ball, suction tool
  - overlay panel (inputs, status, pause/play)

---

### `src/util/math3d.rb`

Small vector helpers and utility functions:

- vector constructors `v2`, `v3`
- `add`, `sub`, `scale`, `length`, `normalize`
- `clamp`, `color_u8`, `rect`

---

### `src/robot/robot_arm.rb`

Robot model and kinematics API:

- `LinkParams`: link length, mass, inertia approximations (rod model)
- `JointAngles`: the 3 joint variables (yaw + 2 pitches)
- `IKResult` and `FKResult`
- `RobotArm`:
  - `solve_ik`
  - `forward_kinematics`
  - reach limits: `min_reach`, `max_reach`

---

### `src/sim/trajectory.rb`

Minimal trajectory generator:

- `Sim::LinearTrajectory`:
  - `reset(from,to,duration)`
  - `update(dt)`
  - `position`
  - `finished?`

---

### `src/ui/overlay.rb`

Overlay panel:

- editable START/GOAL input fields (when paused)
- PLAY applies inputs and starts a new loop
- shows:
  - link lengths, masses, inertias
  - workspace bounds
  - reachability and validation messages

---

### `src/render/draw_utils.rb`

Rendering utilities using raylib primitives:

- `draw_text_bold`, `draw_text_small`
- robot visuals:
  - pedestal + base flange
  - joint housings (sphere + collar)
  - tapered link cylinders with end caps
  - suction tool at the EE

---

<a id="simulation-video"></a>

## Simulation video

Below is a link to the simulation video on YouTube.

<a href="https://www.youtube.com/watch?v=1EnMUH4LTB8" target="_blank">
  <img
    src="https://i.ytimg.com/vi/1EnMUH4LTB8/maxresdefault.jpg"
    alt="3D simulation of an RRR manipulator written in Ruby"
    style="max-width: 100%; border-radius: 10px; box-shadow: 0 6px 18px rgba(0,0,0,0.18); margin-top: 0.5rem;"
  />
</a>

---

<a id="troubleshooting"></a>

## Troubleshooting

### `bundle install` fails

- Make sure you are using a supported Ruby (3.x recommended).
- Update RubyGems and Bundler:
  ```bash
  gem update --system
  gem install bundler
  ```

### The window does not open / crashes immediately on Windows

- Ensure your environment is 64-bit Ruby and you are using the `x64-mingw` gem build.
- If you have multiple Rubies installed, confirm:
  ```bash
  ruby -v
  which ruby
  ```

### UI font looks wrong / fallback font is used

- Confirm the font exists:
  - `resources/fonts/Inter-Regular.ttf`
- If the file is missing or the font fails to load, the UI falls back to the default raylib font or a Windows font.

### START/GOAL out of reach

- Ensure:
  - `z >= 0`
  - `|p|` is within `[|L1 - L2|, L1 + L2]`
- Use PAUSE to edit inputs, then PLAY to apply them.
