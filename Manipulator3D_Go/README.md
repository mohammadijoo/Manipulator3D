<div style="font-family: system-ui, -apple-system, BlinkMacSystemFont, 'Segoe UI', sans-serif; line-height: 1.6;">

  <h1 align="center" style="margin-bottom: 0.2em;">Manipulator3D — 3-DOF Two-Link Robot Arm (Analytic IK + Pick&amp;Place) — Go</h1>

  <p style="font-size: 0.98rem; max-width: 920px; margin: 0.2rem auto 0;">
    A Go simulation and visualization project for a <strong>3-DOF, two-link manipulator</strong> with a <strong>fixed base</strong> in 3D space.
    The arm executes a simple <strong>pick-and-place loop</strong> using an <strong>analytic inverse kinematics</strong> solver and a <strong>linear end-effector trajectory</strong>.
  </p>

  <ul style="max-width: 920px; margin: 0.5rem auto 0.2rem; padding-left: 1.2rem;">
    <li><strong>Joint 0 (base):</strong> revolute yaw about the <strong>+Z axis</strong> at the origin <code>(0,0,0)</code>.</li>
    <li><strong>Joints 1–2 (links):</strong> revolute pitch angles defining motion in the arm’s vertical plane (relative to the <strong>X–Y plane</strong>).</li>
    <li><strong>IK:</strong> reduces the 3D target to a 2D planar problem in <code>(r,z)</code>, solves with the <strong>law of cosines</strong>, and supports elbow-up / elbow-down branches.</li>
    <li><strong>Visualization:</strong> rendered with <strong>raylib-go</strong>, including an overlay panel with editable START/GOAL fields and a suction-tool “ball” pick-and-place animation.</li>
  </ul>

  <p align="center" style="font-size: 1rem; color: #666; margin: 0.35rem auto 0;">
    Build system: <strong>Go modules</strong> • Rendering: <strong>raylib-go</strong> • Language: <strong>Go</strong> • IK: <strong>Closed-form</strong>
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
  <li> <a href="#building-the-project">Building the project</a></li>
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

**User inputs** (via UI overlay panel, not console):
- START position `(x, y, z)` and GOAL position `(x, y, z)` are edited in the overlay **while PAUSED**.
- Press **PLAY** to apply new points and start/restart the loop.
- Constraint enforced by IK: **z must be ≥ 0**.
- Reachability enforced by IK: `|p|` must be within the arm’s reachable shell.

**Fixed HOME EE position**
- HOME is fixed at: **(2, 2, 2)**.

---

<a id="repository-structure"></a>

## Repository structure

```txt
Manipulator3D_Go/
  go.mod
  README.md
  src/
    main.go
    robot/
      robot_model.go
    sim/
      trajectory.go
    ui/
      overlay.go
    render/
      draw_utils.go
  resources/
    fonts/
      Inter-Regular.ttf   (optional)
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
- `|p|` within `[MinReach(), MaxReach()]`

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

These values are displayed in the overlay panel and can be used as a foundation for extending the project to:
- forward dynamics (torques → accelerations),
- gravity and Coriolis terms,
- joint-space controllers (PD, computed torque, etc.).

---

<a id="building-the-project"></a>

## Building the project

### Dependencies (all platforms)

- Go (recommended: install via your platform’s package manager)
- raylib-go (fetched via `go mod`)
- A working C toolchain for cgo (required by raylib-go on most platforms)

### Windows build (recommended): MSYS2 MinGW x64

This section documents the exact setup that avoids common cgo + compiler issues.

#### 1) Install MSYS2

Download and install MSYS2 from the official website:

- https://www.msys2.org/

#### 2) Update MSYS2 core packages (MSYS shell)

Open **“MSYS2 MSYS”** terminal and run:

```bash
pacman -Syu
```

Close the terminal when prompted, reopen **“MSYS2 MSYS”**, then run again:

```bash
pacman -Syu
```

Repeat until there are no pending upgrades.

#### 3) Install MinGW-w64 x64 toolchain (MINGW64 shell)

Open **“MSYS2 MinGW x64”** terminal (important) and verify:

```bash
echo $MSYSTEM
```

Expected output:

```txt
MINGW64
```

Now install the x64 toolchain:

```bash
pacman -S --needed mingw-w64-x86_64-toolchain mingw-w64-x86_64-cmake mingw-w64-x86_64-make mingw-w64-x86_64-pkgconf
```

Verify GCC:

```bash
which gcc
gcc -dumpmachine
gcc --version
```

You should see `x86_64-w64-mingw32`.

#### 4) Install Go inside MSYS2 (MINGW64 shell)

Still in **“MSYS2 MinGW x64”**, install Go:

```bash
pacman -S --needed mingw-w64-x86_64-go
```

Verify:

```bash
which go
go version
```

#### 5) Build commands (MINGW64 shell)

From the project root:

```bash
cd /d/Manipulator3D_Go
go mod tidy
go clean -cache
go build -o Manipulator3D.exe ./src
```

**Important notes**
- Build in **MINGW64**, not in Git Bash, to ensure cgo uses the correct compiler and libraries.
- If you move the project folder, adjust `cd` accordingly (MSYS2 paths use `/d/...` for drive D:).

### Linux (general)

Install a Go toolchain and the C build essentials for your distro (examples):

- Debian/Ubuntu:
  ```bash
  sudo apt-get update
  sudo apt-get install -y build-essential pkg-config
  ```
- Fedora:
  ```bash
  sudo dnf install -y @development-tools pkgconf-pkg-config
  ```

Then build:

```bash
go mod tidy
go clean -cache
go build -o Manipulator3D ./src
```

### macOS (general)

Install Xcode Command Line Tools:

```bash
xcode-select --install
```

Then build:

```bash
go mod tidy
go clean -cache
go build -o Manipulator3D ./src
```

---

<a id="running-the-simulator"></a>

## Running the simulator

### Run command (Windows / MINGW64)

From the project root:

```bash
./Manipulator3D.exe
```

### Run command (Linux/macOS)

```bash
./Manipulator3D
```

### Controls / interaction

- Overlay panel includes **PAUSE/PLAY**.
- Edit START/GOAL fields while **PAUSED**.
- Press **PLAY** to apply new points and start the loop.
- Mouse wheel: zoom camera
- F11: toggle fullscreen

---

<a id="repository-file-guide"></a>

## Repository file guide (full explanation)

This section explains every important file in the repository and its role.

### `go.mod`

Key responsibilities:

- defines the module name and Go version
- pins raylib-go dependency so builds are reproducible

---

### `src/main.go`

Application entry point and runtime loop:

- initializes the robot model (link lengths, masses, inertias)
- maintains the pick-and-place finite-state machine:
  - HOME → START → PICK → GOAL → PLACE → HOME → WAIT → LOOP
- generates a **linear Cartesian trajectory** between targets
- runs IK each frame to get joint angles for the current EE target
- calls FK for rendering joint/link positions
- renders:
  - robot geometry, axes, ball, suction tool
  - overlay panel (editable start/goal, status, pause/play)

---

### `src/robot/robot_model.go`

Robot model and kinematics:

- `LinkParams`: link length, mass, inertia approximations (rod model)
- `JointAngles`: the 3 joint variables (yaw + 2 pitches)
- `FKResult`: base/joints/EE positions for rendering
- `IKResult`: reachability + solution angles + message
- `RobotArm`:
  - `SolveIK()`
  - `ForwardKinematics()`
  - reach limits: `MinReach()`, `MaxReach()`

---

### `src/sim/trajectory.go`

Minimal trajectory generator:

- `LinearTrajectory`:
  - `Reset(from,to,duration)`
  - `Update(dt)`
  - `Position()`
  - `Finished()`

---

### `src/ui/overlay.go`

UI overlay rendering + editable numeric fields:

- Draws a panel containing:
  - PAUSE/PLAY button
  - editable START/GOAL `(x,y,z)` fields (while paused)
  - link parameters and workspace bounds
  - reachability diagnostics and phase text
- Sends actions back to `main.go`:
  - updated pause/play state
  - “apply new start/goal” event when PLAY is pressed while paused

---

### `src/render/draw_utils.go`

Rendering helpers using raylib primitives:

- text helpers:
  - `DrawTextBold()`
  - `DrawTextSmall()`
- robot visuals:
  - pedestal + base flange
  - joint housings (sphere + collar)
  - tapered link cylinders with end caps
  - suction tool at the EE

---

<a id="simulation-video"></a>

## Simulation video

Below is a link to the simulation video on YouTube.

<a href="https://www.youtube.com/watch?v=sawyzknLW8c" target="_blank">
  <img
    src="https://i.ytimg.com/vi/sawyzknLW8c/maxresdefault.jpg"
    alt="3D simulation of an RRR manipulator in Go"
    style="max-width: 100%; border-radius: 10px; box-shadow: 0 6px 18px rgba(0,0,0,0.18); margin-top: 0.5rem;"
  />
</a>

---

<a id="troubleshooting"></a>

## Troubleshooting

### `runtime/cgo: ... cgo.exe: exit status 2` on Windows

This usually indicates cgo cannot find or cannot use a compatible C toolchain.

Recommended resolution:
1) Install MSYS2 and fully update it (see **Windows build: MSYS2 MinGW x64**).
2) Build in **“MSYS2 MinGW x64”** (MINGW64), not in Git Bash.
3) Verify in MINGW64:
   ```bash
   echo $MSYSTEM
   which gcc
   which go
   ```
   Expected: `MINGW64`, `/mingw64/bin/gcc`, `/mingw64/bin/go`

### Dependency/version conflicts during `pacman` installs

If `pacman` reports dependency conflicts (common after partial upgrades), do a full upgrade in the **MSYS** terminal:

```bash
pacman -Syu
```

Close/reopen as prompted and run again until fully updated.

### `build constraints exclude all Go files` under `src/robot`

Avoid using file names that end with `_arm.go` (that suffix is treated as an architecture-specific file name pattern in Go toolchains). Use a neutral file name such as:

- `robot_model.go`

### No executable produced (`./Manipulator3D.exe: No such file or directory`)

If the build fails, the executable will not exist. Always check the build output first:

```bash
go build -o Manipulator3D.exe ./src
```

### Start/Goal is “out of reach”

Ensure:
- `z >= 0`
- `|p|` is within the workspace shell:
  - `[|L1 - L2|, L1 + L2]`

---
