<div style="font-family: system-ui, -apple-system, BlinkMacSystemFont, 'Segoe UI', sans-serif; line-height: 1.6;">

  <h1 align="center" style="margin-bottom: 0.2em;">Manipulator3D — 3-DOF Two-Link Robot Arm (Analytic IK + Pick&amp;Place) — Scala</h1>

  <p style="font-size: 0.98rem; max-width: 920px; margin: 0.2rem auto 0;">
    A Scala simulation and visualization project for a <strong>3-DOF, two-link manipulator</strong> with a <strong>fixed base</strong> in 3D space.
    The arm executes a simple <strong>pick-and-place loop</strong> using an <strong>analytic inverse kinematics</strong> solver and a <strong>linear end-effector trajectory</strong>.
  </p>

  <ul style="max-width: 920px; margin: 0.5rem auto 0.2rem; padding-left: 1.2rem;">
    <li><strong>Joint 0 (base):</strong> revolute yaw about the <strong>+Z axis</strong> at the origin <code>(0,0,0)</code>.</li>
    <li><strong>Joints 1–2 (links):</strong> revolute pitch angles defining motion in the arm’s vertical plane (relative to the <strong>X–Y plane</strong>).</li>
    <li><strong>IK:</strong> reduces the 3D target to a 2D planar problem in <code>(r,z)</code>, solves with the <strong>law of cosines</strong>, and supports elbow-up / elbow-down branches.</li>
    <li><strong>Visualization:</strong> rendered with <strong>raylib</strong> via the <strong>jaylib</strong> Java bindings, including an overlay panel and a suction-tool “ball” pick-and-place animation.</li>
    <li><strong>User interaction:</strong> START and GOAL targets are entered in the <strong>overlay panel</strong> (not the console). Press <strong>PLAY</strong> to run and <strong>PAUSE</strong> to edit targets.</li>
  </ul>

  <p align="center" style="font-size: 1rem; color: #666; margin: 0.35rem auto 0;">
    Build system: <strong>sbt</strong> • Rendering: <strong>raylib (jaylib)</strong> • Language: <strong>Scala</strong> • IK: <strong>Closed-form</strong>
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

**User inputs** (via overlay panel in the window):
- START position `(x, y, z)` and GOAL position `(x, y, z)`
- Constraint enforced by IK: **z must be ≥ 0**
- Reachability enforced by IK: `|p|` must be within the arm’s reachable shell
- Interaction:
  - Start the simulation by pressing **PLAY**
  - Press **PAUSE** to edit START/GOAL, then **PLAY** again

---

<a id="repository-structure"></a>

## Repository structure

```txt
Manipulator3D_Scala/
  build.sbt
  project/
    build.properties
  README.md
  resources/
    fonts/
      LICENSE.txt
      DOWNLOAD_FONT.md
  src/
    main/
      scala/
        manipulator3d/
          Main.scala
          math/
            Vec3.scala
          robot/
            RobotArm.scala
          sim/
            Trajectory.scala
          ui/
            Overlay.scala
          render/
            DrawUtils.scala
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
- `manipulator3d.robot.RobotArm.forwardKinematics()`

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
- `|p|` within `[minReach, maxReach]`

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
- `manipulator3d.robot.RobotArm.solveIK()`

### 5) Dynamics (what is implemented vs. what is prepared)

This repository is primarily a **kinematic + trajectory** simulator:

- The EE follows a commanded Cartesian trajectory.
- IK converts EE targets into joint angles.
- FK is used for rendering joint/link positions.

However, the code also defines **mass and inertia properties** for each link (uniform rod approximations):

- About center of mass:

```math
I_{cm} = \frac{1}{12} mL^2
```

- About the joint at one end:

```math
I_{joint} = \frac{1}{3} mL^2
```

These are computed in:
- `manipulator3d.robot.LinkParams.recomputeInertia()`

---

<a id="building-the-project"></a>

## Building the project

### Dependencies

- Java **17+**
- sbt **1.12.x** (this repo pins the launcher in `project/build.properties`)  
- The project depends on **jaylib** (Raylib 5.5 bindings for Java) pulled from Maven repositories by sbt.  
  - Maven/Gradle coordinates and platform notes are described in the jaylib README. citeturn1view0
- Optional: you can place `Inter-Regular.ttf` at `resources/fonts/Inter-Regular.ttf` for a consistent UI font (fallback to system fonts is implemented).

### Build & run (sbt)

From the repository root:

```bash
sbt clean run
```

**macOS note:** raylib window creation may require running on the first thread. If you are on macOS, use:

```bash
sbt -J-XstartOnFirstThread run
```

The jaylib README explicitly documents this requirement. citeturn1view0

---

<a id="running-the-simulator"></a>

## Running the simulator

When the window opens:

- Use the overlay panel to set:
  - START `(x, y, z)`
  - GOAL `(x, y, z)`
- Press **PLAY** to start the pick-and-place loop
- Press **PAUSE** to freeze the simulation, edit START/GOAL, then press **PLAY** again
- Mouse wheel: zoom camera
- F11: toggle fullscreen

**Important constraints:**
- `z >= 0` is enforced for both targets
- both targets must be reachable (`|p|` within workspace)

---

<a id="repository-file-guide"></a>

## Repository file guide (full explanation)

### `build.sbt`

Key responsibilities:

- sets Scala version and project metadata
- pulls in jaylib dependency:
  - `uk.co.electronstudio.jaylib:jaylib:5.5.0-2` citeturn1view0
- configures `sbt run` entry point: `manipulator3d.Main`
- forks a new JVM for running (recommended for native libraries)

---

### `src/main/scala/manipulator3d/Main.scala`

Application entry point and runtime loop:

- creates the robot model and camera
- hosts the pick-and-place finite-state machine:
  - HOME → START → PICK → GOAL → PLACE → HOME → WAIT → LOOP
- generates a **linear Cartesian trajectory** between targets
- runs IK each frame to get joint angles for the current EE target
- calls FK for rendering joint/link positions
- renders:
  - robot geometry, axes, ball, suction tool
  - overlay panel (status + PLAY/PAUSE + target editing)

---

### `src/main/scala/manipulator3d/robot/RobotArm.scala`

Declares and implements the robot model and kinematics:

- `LinkParams`: link length, mass, inertia approximations (rod model)
- `JointAngles`: yaw + two pitches
- `solveIK()`: closed-form IK with reachability checks
- `forwardKinematics()`: positions of elbow and end-effector

---

### `src/main/scala/manipulator3d/sim/Trajectory.scala`

Minimal trajectory generator:

- `LinearTrajectory.reset(from,to,duration)`
- `LinearTrajectory.update(dt)`
- `LinearTrajectory.position`
- `LinearTrajectory.isFinished`

---

### `src/main/scala/manipulator3d/ui/Overlay.scala`

Overlay panel and text-input handling:

- input boxes for START/GOAL coordinates
- clickable PLAY/PAUSE button
- reachability feedback and basic validation messages

---

### `src/main/scala/manipulator3d/render/DrawUtils.scala`

Rendering helpers using raylib primitives:

- bold/small text helpers
- robot visuals:
  - pedestal + joint housings
  - tapered link cylinders
  - suction tool at the end-effector

---

### `resources/fonts/`

This folder can optionally contain `Inter-Regular.ttf` for consistent UI rendering. 

---

<a id="simulation-video"></a>

## Simulation video

Below is a link to the simulation video on YouTube.

<a href="https://www.youtube.com/watch?v=5nmAPRZK6U8" target="_blank">
  <img
    src="https://i.ytimg.com/vi/5nmAPRZK6U8/maxresdefault.jpg"
    alt="3D simulation of an RRR manipulator in Scala"
    style="max-width: 100%; border-radius: 10px; box-shadow: 0 6px 18px rgba(0,0,0,0.18); margin-top: 0.5rem;"
  />
</a>

---

<a id="troubleshooting"></a>

## Troubleshooting

### `sbt run` fails to load the native library / window does not appear

- Make sure you are using a supported OS/architecture for the jaylib native binaries. The jaylib README lists supported platforms. citeturn1view0
- On macOS, run with:
  ```bash
  sbt -J-XstartOnFirstThread run
  ```
  as documented by jaylib. citeturn1view0

### Start/Goal is “out of reach”

- Ensure:
  - `z >= 0`
  - `|p|` is within the workspace shell:
    - `[|L1 - L2|, L1 + L2]`

### Font not found

- The code tries, in order:
  - `resources/fonts/Inter-Regular.ttf`
  - common Windows fonts (`segoeui.ttf`, `arial.ttf`)
  - a common Linux font (`DejaVuSans.ttf`)
- You can swap in any `.ttf` by changing the candidates list in `Main.scala`.
