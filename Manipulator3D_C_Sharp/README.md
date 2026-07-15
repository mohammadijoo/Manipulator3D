<div style="font-family: system-ui, -apple-system, BlinkMacSystemFont, 'Segoe UI', sans-serif; line-height: 1.6;">

  <h1 align="center" style="margin-bottom: 0.2em;">Manipulator3D — 3-DOF Two-Link Robot Arm (Analytic IK + Pick&amp;Place)</h1>

  <p style="font-size: 0.98rem; max-width: 920px; margin: 0.2rem auto 0;">
    A C# simulation and visualization project for a <strong>3-DOF, two-link manipulator</strong> with a <strong>fixed base</strong> in 3D space.
    The arm executes a simple <strong>pick-and-place loop</strong> using an <strong>analytic inverse kinematics</strong> solver and a <strong>linear end-effector trajectory</strong>.
  </p>

  <ul style="max-width: 920px; margin: 0.5rem auto 0.2rem; padding-left: 1.2rem;">
    <li><strong>Joint 0 (base):</strong> revolute yaw about the <strong>+Z axis</strong> at the origin <code>(0,0,0)</code>.</li>
    <li><strong>Joints 1–2 (links):</strong> revolute pitch angles defining motion in the arm’s vertical plane (relative to the <strong>X–Y plane</strong>).</li>
    <li><strong>IK:</strong> reduces the 3D target to a 2D planar problem in <code>(r,z)</code>, solves with the <strong>law of cosines</strong>, and supports elbow-up / elbow-down branches.</li>
    <li><strong>Visualization:</strong> rendered with <strong>raylib</strong> via <strong>Raylib-cs</strong>, including an overlay panel and a suction-tool “ball” pick-and-place animation.</li>
  </ul>

  <p align="center" style="font-size: 1rem; color: #666; margin: 0.35rem auto 0;">
    IDE: <strong>Visual Studio 2022</strong> • Runtime: <strong>.NET</strong> • Rendering: <strong>raylib (Raylib-cs)</strong> • IK: <strong>Closed-form</strong>
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
  <li> <a href="#running-in-visual-studio-2022">Running in Visual Studio 2022</a></li>
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

**User inputs** (via overlay UI):
- START position `(x, y, z)` and GOAL position `(x, y, z)` are entered directly inside the UI panel.
- Constraint enforced by IK: **z must be ≥ 0**
- Reachability enforced by IK: `|p|` must be within the arm’s reachable shell.
- Press **PLAY** to start; press **PAUSE** to edit points and run again.

---

<a id="repository-structure"></a>

## Repository structure

```txt
Manipulator3D/
  Manipulator3D.sln
  Manipulator3D.csproj
  README.md
  src/
    Program.cs
    RaylibCompat.cs
    robot/
      RobotArm.cs
    sim/
      Trajectory.cs
    ui/
      Overlay.cs
    render/
      DrawUtils.cs
  resources/
    fonts/
      Inter-Regular.ttf
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

This is implemented in:
- `robot/RobotArm.cs` → `RobotArm.ForwardKinematics(...)`

### 3) Workspace (reachability) check

This manipulator is effectively a 2-link arm in a vertical plane, rotated by yaw. Therefore the reachable radius must satisfy:

```math
r_{\min} = |L_1 - L_2|,\quad r_{\max} = L_1 + L_2
```

```math
\|\mathbf{p}\| \in [r_{\min}, r_{\max}]
```

The implementation enforces:
- `target.Z >= 0`
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

Now solve a 2-link planar problem for the triangle formed by $L_1$, $L_2$, and $\sqrt{r^2+z^2}$.

**Step C — elbow angle from law of cosines**

```math
\cos(q_2) = \frac{r^2 + z^2 - L_1^2 - L_2^2}{2L_1L_2}
```

Clamp to $[-1,1]$ for numerical safety, then:

```math
q_2 = \pm \arccos(\cos(q_2))
```

This sign is the **elbow-up / elbow-down** branch selection.

**Step D — shoulder angle**

Define:

```math
k_1 = L_1 + L_2\cos(q_2),\quad k_2 = L_2\sin(q_2)
```

Then:

```math
q_1 = \mathrm{atan2}(z, r) - \mathrm{atan2}(k_2, k_1)
```

This is implemented in:
- `robot/RobotArm.cs` → `RobotArm.SolveIK(...)`

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

These are computed in:
- `robot/RobotArm.cs` → `LinkParams.RecomputeInertia()`

---

<a id="building-the-project"></a>

## Building the project

### Dependencies

- **.NET SDK** (recommended: .NET 8 or newer)
- NuGet packages are restored automatically by Visual Studio or `dotnet restore`
- raylib bindings are provided via **Raylib-cs**

### Command-line build (optional)

```bash
dotnet restore
dotnet build -c Release
```

---

<a id="running-in-visual-studio-2022"></a>

## Running in Visual Studio 2022

If your repository contains **`Manipulator3D.sln`**, the simplest workflow is:

1) **Open the solution**
- Double-click **`Manipulator3D.sln`**
- Or use Visual Studio: **File → Open → Project/Solution…** and select the `.sln`

2) **Restore packages** (usually automatic)
- Visual Studio will restore NuGet dependencies automatically.
- If it doesn’t, right-click the solution → **Restore NuGet Packages**.

3) **Select configuration**
- Top toolbar: choose **Debug** (for development) or **Release** (for performance).

4) **Run**
- Click the green **Start** button, or press **F5**.
- The simulator window opens immediately.

**Tip (assets like fonts):**
- The font file is expected at: `resources/fonts/Inter-Regular.ttf`
- Ensure it is copied to the output folder (the project typically includes a “Copy to Output Directory” rule).

---

<a id="running-the-simulator"></a>

## Running the simulator

### Controls / interaction

- Mouse wheel: zoom camera
- F11: toggle fullscreen
- Overlay panel:
  - Enter START and GOAL values
  - **PLAY** starts the loop
  - **PAUSE** stops simulation so you can enter new values

---

<a id="repository-file-guide"></a>

## Repository file guide (full explanation)

This section explains every important file in the repository and its role.

### `Manipulator3D.sln`

- Visual Studio solution file for opening/running the project easily.

### `Manipulator3D.csproj`

- Project configuration, NuGet references, and content-copy rules (e.g., fonts in `resources/`).

---

### `src/Program.cs`

This is the application entry point and runtime loop:

- reads START and GOAL targets from the overlay UI
- validates reachability using IK for:
  - fixed HOME EE point
  - START
  - GOAL
- defines a pick-and-place finite-state machine:
  - HOME → START → PICK → GOAL → PLACE → HOME → WAIT → LOOP
- generates a **linear Cartesian trajectory** between targets
- runs IK each frame to get joint angles for the current EE target
- calls FK for rendering joint/link positions
- renders:
  - robot geometry, axes, ball, suction tool
  - overlay panel (inputs, status, pause/play)

---

### `src/robot/RobotArm.cs`

Declares and implements the robot model and kinematics API:

- `LinkParams`: link length, mass, inertia approximations (rod model)
- `JointAngles`: the 3 joint variables (yaw + 2 pitches)
- `FKResult`: base/joints/EE positions for rendering
- `IKResult`: reachability + solution angles + message
- `RobotArm` class:
  - `SolveIK()`
  - `ForwardKinematics()`
  - reach limits: `MinReach()`, `MaxReach()`

---

### `src/sim/Trajectory.cs`

Implements a minimal linear trajectory generator:

- `LinearTrajectory`:
  - `Reset(from,to,duration)`
  - `Update(dt)`
  - `Position()`
  - `Finished()`

---

### `src/ui/Overlay.cs`

Implements the overlay UI:

- input fields for START/GOAL
- play/pause button
- displays:
  - link lengths, masses, inertias
  - workspace bounds
  - reachability warnings and runtime errors

---

### `src/render/DrawUtils.cs`

Rendering helpers built on raylib primitives:

- text helpers:
  - `DrawTextBold()`
  - `DrawTextSmall()`
- robot visuals:
  - pedestal + flange
  - joint housings
  - tapered link cylinders + caps
  - suction tool

---

<a id="simulation-video"></a>

## Simulation video

Below is a link to a simulation video on YouTube.

<a href="https://www.youtube.com/watch?v=pP8kgVg1SNE" target="_blank">
  <img
    src="https://i.ytimg.com/vi/pP8kgVg1SNE/maxresdefault.jpg"
    alt="3D simulation of an RRR manipulator in C#"
    style="max-width: 100%; border-radius: 10px; box-shadow: 0 6px 18px rgba(0,0,0,0.18); margin-top: 0.5rem;"
  />
</a>

---

<a id="troubleshooting"></a>

## Troubleshooting

### Visual Studio builds but run fails due to missing assets (fonts, etc.)

- Ensure `resources/fonts/Inter-Regular.ttf` exists in the repo.
- Ensure the `.csproj` copies `resources/` to the output directory.

### Start/Goal is “out of reach”

- Ensure:
  - `z >= 0`
  - `|p|` is within the workspace shell:
    - `[|L1 - L2|, L1 + L2]`
