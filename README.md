<h1 align="center">Manipulator3D — 3-DOF Two-Link Robot Arm (Analytic IK + Pick&amp;Place)</h1>

A Java simulation and visualization project for a **3-DOF, two-link manipulator** with a **fixed base** in 3D space.
The arm executes a simple **pick-and-place loop** using an **analytic inverse kinematics** solver and a **linear end-effector trajectory**.

- **Joint 0 (base):** revolute yaw about the **+Z axis** at the origin `(0,0,0)`.
- **Joints 1–2 (links):** revolute pitch angles defining motion in the arm’s vertical plane (relative to the **X–Y plane**).
- **IK:** reduces the 3D target to a 2D planar problem in `(r,z)`, solves with the **law of cosines**, and supports elbow-up / elbow-down branches.
- **Visualization:** rendered with **raylib** via **Jaylib**, including an overlay panel and a suction-tool “ball” pick-and-place animation.

<p align="center">
  Build system: <strong>Gradle</strong> • Rendering: <strong>raylib (Jaylib)</strong> • Language: <strong>Java 17</strong> • IK: <strong>Closed-form</strong>
</p>

---

## Table of Contents

- [About this repository](#about-this-repository)
- [Repository structure](#repository-structure)
- [Robotics: kinematics, dynamics, and inverse kinematics](#robotics-kinematics-dynamics-and-ik)
- [Building the project](#building-the-project)
- [Running the simulator](#running-the-simulator)
- [Repository file guide (full explanation)](#repository-file-guide-full-explanation)
- [Simulation video](#simulation-video)
- [Troubleshooting](#troubleshooting)

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

**User inputs** (via console prompt):
- START position `(x y z)` and GOAL position `(x y z)`
- Constraint enforced by IK: **z must be ≥ 0**
- Reachability enforced by IK: `|p|` must be within the arm’s reachable shell.

---

<a id="repository-structure"></a>

## Repository structure

```txt
Manipulator3D_Java/
  build.gradle
  settings.gradle
  README.md
  resources/
    fonts/
      Inter-Regular.ttf   (optional)
  src/
    main/
      java/
        manipulator3d/
          Main.java
          robot/
            Vec3f.java
            LinkParams.java
            JointAngles.java
            FKResult.java
            IKResult.java
            RobotArm.java
          sim/
            LinearTrajectory.java
          ui/
            Overlay.java
          render/
            DrawUtils.java
```

---

<a id="robotics-kinematics-dynamics-and-ik"></a>

## Robotics: kinematics, dynamics, and inverse kinematics

### 1) Coordinate frames and joint conventions

- World frame origin is at the center of the base revolute joint: **(0,0,0)**.
- Joint angles:
  - `q0Yaw` : rotation about **+Z** (sets the arm’s radial direction in the X–Y plane)
  - `q1Pitch`: shoulder elevation angle relative to the **X–Y plane**
  - `q2Pitch`: elbow pitch angle (relative bend in the same vertical plane)

We use the radial unit direction in the X–Y plane:

$$
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
$$

### 2) Forward kinematics (FK)

Let link lengths be $L_1$ and $L_2$. Then:

- Elbow position:

$$
\mathbf{p}_1 = L_1\cos(q_1)\,\mathbf{u} + L_1\sin(q_1)\,\mathbf{k}
$$

- End-effector position:

$$
\mathbf{p}_{ee} = \mathbf{p}_1 + L_2\cos(q_1+q_2)\,\mathbf{u} + L_2\sin(q_1+q_2)\,\mathbf{k}
$$

This is implemented in:
- `manipulator3d.robot.RobotArm#forwardKinematics(...)`

### 3) Workspace (reachability) check

This manipulator is effectively a 2-link arm in a vertical plane, rotated by yaw. Therefore the reachable radius must satisfy:

$$
r_{\min} = |L_1 - L_2|,\quad r_{\max} = L_1 + L_2
$$

$$
\|\mathbf{p}\| \in [r_{\min}, r_{\max}]
$$

The implementation enforces:
- `target.z >= 0`
- `|p|` within `[minReach(), maxReach()]`

### 4) Analytic inverse kinematics (IK): how it works here

Given target $\mathbf{p} = (x,y,z)$:

**Step A — base yaw**

$$
q_0 = \mathrm{atan2}(y, x)
$$

**Step B — reduce to planar IK in $(r,z)$**

$$
r = \sqrt{x^2 + y^2}
$$

Now solve a 2-link planar problem for the triangle formed by $L_1$, $L_2$, and $\sqrt{r^2+z^2}$.

**Step C — elbow angle from law of cosines**

$$
\cos(q_2) = \frac{r^2 + z^2 - L_1^2 - L_2^2}{2L_1L_2}
$$

Clamp to $[-1,1]$ for numerical safety, then:

$$
q_2 = \pm \arccos(\cos(q_2))
$$

This sign is the **elbow-up / elbow-down** branch selection.

**Step D — shoulder angle**

Define:

$$
k_1 = L_1 + L_2\cos(q_2),\quad k_2 = L_2\sin(q_2)
$$

Then:

$$
q_1 = \mathrm{atan2}(z, r) - \mathrm{atan2}(k_2, k_1)
$$

This is implemented in:
- `manipulator3d.robot.RobotArm#solveIK(...)`

**Practical note (this repo):**
- The main program uses the default elbow branch (elbow-down) by passing `elbowUp = false`.

### 5) Dynamics (what is implemented vs. what is prepared)

This repository is primarily a **kinematic + trajectory** simulator:

- The EE follows a commanded Cartesian trajectory.
- IK converts EE targets into joint angles.
- FK is used for rendering joint/link positions.

However, the code also defines **mass and inertia properties** for each link (uniform rod approximations):

- About center of mass:

$$
I_{cm} = \frac{1}{12} mL^2
$$

- About the joint at one end:

$$
I_{joint} = \frac{1}{3} mL^2
$$

These are computed in:
- `manipulator3d.robot.LinkParams#recomputeInertia()`

Currently, those values are used for **display/inspection** (overlay panel) and as a foundation for extending the project to:
- forward dynamics (torques → accelerations),
- gravity and Coriolis terms,
- joint-space controllers (PD, computed torque, etc.).

---

<a id="building-the-project"></a>

## Building the project

### Dependencies

- Java **17+** (JDK)
- Gradle **7+** (Gradle 8+ recommended)
- raylib bindings are resolved automatically through Gradle via **Jaylib**

### Build commands

From the project root:

```bash
gradle build
```

---

<a id="running-the-simulator"></a>

## Running the simulator

### Run command

From the project root:

```bash
gradle run
```

**macOS note:**
- Some macOS configurations require starting Java on the first thread. If you see a startup error, run with:
  ```bash
  gradle run --jvm-args="-XstartOnFirstThread"
  ```

### Controls / interaction

- At startup, the console asks for:
  - START `(x y z)` and GOAL `(x y z)`
- Mouse wheel: zoom camera
- F11: toggle fullscreen
- Overlay includes a **PAUSE/PLAY** button and reachability feedback

---

<a id="repository-file-guide-full-explanation"></a>

## Repository file guide (full explanation)

This section explains every important file in the repository and its role.

### `build.gradle`

Key responsibilities:

- configures a Java 17 application
- fetches **Jaylib** from Maven Central
- sets the main entry point: `manipulator3d.Main`

---

### `src/main/java/manipulator3d/Main.java`

This is the application entry point and runtime loop:

- prompts user for START and GOAL targets
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
  - robot geometry, thick axes, ball, suction tool
  - overlay panel (status, parameters, pause button)

---

### `src/main/java/manipulator3d/robot/RobotArm.java`

Implements analytic FK and IK:

- **IK**:
  - rejects invalid targets (z < 0)
  - checks radius bounds (min/max reach)
  - computes yaw from `atan2(y,x)`
  - reduces to planar IK in `(r,z)`
  - solves elbow angle via law of cosines
  - solves shoulder angle via triangle decomposition
- **FK**:
  - constructs radial axis from yaw
  - builds elbow and EE positions from link lengths and pitch angles

---

### `src/main/java/manipulator3d/sim/LinearTrajectory.java`

Implements linear interpolation in 3D:

- uses a normalized time parameter: $\alpha = \mathrm{clamp}(t/T, 0, 1)$
- outputs: $\mathbf{p}(\alpha) = \mathbf{a} + (\mathbf{b}-\mathbf{a})\alpha$

---

### `src/main/java/manipulator3d/ui/Overlay.java`

Implements the overlay rendering:

- draws a semi-transparent panel
- displays:
  - link lengths, masses, inertias
  - workspace bounds
  - start/goal norms and coordinates
  - phase text and reachability warnings
- implements a clickable PAUSE/PLAY button

---

### `src/main/java/manipulator3d/render/DrawUtils.java`

Implements rendering utilities using raylib primitives:

- text helpers:
  - `drawTextBold()`
  - `drawTextSmall()`
- “machined” robot look:
  - pedestal + base flange
  - joint housings (sphere + collar)
  - tapered link cylinders with end caps
  - suction tool at the EE

---

<a id="simulation-video"></a>

## Simulation video

Below is a link to the simulation video on YouTube.

<a href="https://www.youtube.com/watch?v=9-B7WbkG7cM" target="_blank">
  <img
    src="https://i.ytimg.com/vi/9-B7WbkG7cM/maxresdefault.jpg"
    alt="3D simulation of an RRR manipulator"
    style="max-width: 100%; border-radius: 10px; box-shadow: 0 6px 18px rgba(0,0,0,0.18); margin-top: 0.5rem;"
  />
</a>

---

<a id="troubleshooting"></a>

## Troubleshooting

### Jaylib can’t load natives / app fails to start

- Make sure you are running a supported desktop platform (Windows/macOS/Linux x86_64 or ARM64).
- If you are on macOS, try starting with:
  - `-XstartOnFirstThread` (see “Running the simulator”).

### Start/Goal is “out of reach”

- Ensure:
  - `z >= 0`
  - `|p|` is within the workspace shell:
    - `[|L1 - L2|, L1 + L2]`
