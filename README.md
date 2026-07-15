# Manipulator3D — One RRR Robot, Nine Programming Languages

A comparative robotics and graphics project that implements the same **3-DOF RRR manipulator pick-and-place simulation** in nine programming languages.

Every implementation uses **raylib** or a language-specific raylib binding for real-time 3D visualization. The robotics model, inverse-kinematics method, trajectory logic, and pick-and-place sequence are shared across the projects, while the code organization, dependency manager, build system, and language idioms remain native to each ecosystem.

> **RRR** means that all three joints are revolute: one base-yaw joint and two pitch joints. The arm has two serial moving links after the base rotation.

---

## Why This Repository Exists

These implementations were originally developed as separate repositories. Because they represent the same robotics problem and follow the same overall computational process, they are now collected in one repository.

This monorepo makes it easier to:

- study the same robotics algorithm in different programming languages;
- compare syntax, project structure, package management, and graphics bindings;
- explore how mathematical models are translated across language ecosystems;
- clone all implementations once and run each project independently;
- maintain shared conceptual documentation in one location.

The implementations are **conceptually equivalent**, but they are not expected to be line-by-line translations. Language-specific APIs, raylib bindings, build systems, and some interface details differ. For example, some versions accept START and GOAL coordinates through the graphical overlay, while others use console input.

---

## Implementations

| Language | Project folder | Project/build system | raylib integration |
|---|---|---|---|
| C++ | [`Manipulator3D_CPP`](./Manipulator3D_CPP/) | CMake, C++17 | Native raylib |
| Rust | [`Manipulator3D_Rust`](./Manipulator3D_Rust/) | Cargo, Rust 2021 | Rust raylib binding |
| Ruby | [`Manipulator3D_Ruby`](./Manipulator3D_Ruby/) | Bundler | `raylib-bindings` |
| Julia | [`Manipulator3D_Julia`](./Manipulator3D_Julia/) | Julia project environment | Julia raylib binding |
| Go | [`Manipulator3D_Go`](./Manipulator3D_Go/) | Go modules | `raylib-go` |
| Scala | [`Manipulator3D_Scala`](./Manipulator3D_Scala/) | sbt | Jaylib |
| C# | [`Manipulator3D_C_Sharp`](./Manipulator3D_C_Sharp/) | .NET / Visual Studio | Raylib-cs |
| Python | [`Manipulator3D_Python`](./Manipulator3D_Python/) | `pip` / `requirements.txt` | Python raylib binding |
| Java | [`Manipulator3D_Java`](./Manipulator3D_Java/) | Gradle, Java 17 | Jaylib |

Each folder is a self-contained project with its own source code, dependencies, build configuration, resources, and detailed README.

---

## Clone and Run an Implementation

Clone the complete repository:

```bash
git clone https://github.com/abolfazl-mohammadijoo/Manipulator3D.git
cd Manipulator3D
```

Enter the implementation you want to use:

```bash
cd Manipulator3D_CPP
```

Then follow the build and run instructions in that folder's `README.md`.

For another implementation, return to the repository root and enter its folder:

```bash
cd ..
cd Manipulator3D_Rust
```

There is no requirement to build all nine projects. They are independent applications and can be installed, compiled, and executed separately.

---

## Repository Structure

```text
Manipulator3D/
├── README.md
├── Manipulator3D_CPP/
├── Manipulator3D_Rust/
├── Manipulator3D_Ruby/
├── Manipulator3D_Julia/
├── Manipulator3D_Go/
├── Manipulator3D_Scala/
├── Manipulator3D_C_Sharp/
├── Manipulator3D_Python/
└── Manipulator3D_Java/
```

The root of the repository is an index and conceptual guide. Language-specific technical instructions belong inside the corresponding project folder.

---

# Robotics Model

## Manipulator Configuration

The simulated robot is a fixed-base, three-degree-of-freedom RRR manipulator:

1. **Base joint, `q0`**  
   Revolute yaw about the world `+Z` axis.

2. **Shoulder joint, `q1`**  
   Revolute pitch in the arm's vertical plane.

3. **Elbow joint, `q2`**  
   Revolute pitch in the same vertical plane.

The base yaw selects a radial direction in the horizontal `X-Y` plane. The shoulder and elbow then form a conventional two-link planar arm in the resulting vertical `(r, z)` plane.

Let the link lengths be:

- `L1`: first-link length;
- `L2`: second-link length.

---

## Coordinate Reduction

For a Cartesian target

```text
p = (x, y, z)
```

the horizontal radial distance is

```math
r = \sqrt{x^2+y^2}
```

and the base-yaw angle is

```math
q_0 = \mathrm{atan2}(y,x)
```

After computing `q0`, the remaining inverse-kinematics problem is reduced from three-dimensional Cartesian space to a two-link planar problem in `(r, z)`.

---

## Analytic Inverse Kinematics

The elbow angle is obtained from the law of cosines:

```math
D =
\frac{r^2+z^2-L_1^2-L_2^2}
     {2L_1L_2}
```

For numerical safety, `D` is clamped to `[-1, 1]`.

```math
q_2 = \pm\arccos(D)
```

The sign selects the elbow-up or elbow-down branch.

Define

```math
k_1=L_1+L_2\cos(q_2)
```

```math
k_2=L_2\sin(q_2)
```

The shoulder angle is then

```math
q_1 =
\mathrm{atan2}(z,r)
-
\mathrm{atan2}(k_2,k_1)
```

This closed-form solver is evaluated as the end effector follows its commanded Cartesian path.

---

## Forward Kinematics

Define the radial unit vector in the horizontal plane:

```math
\mathbf{u} =
\begin{bmatrix}
\cos(q_0)\\
\sin(q_0)\\
0
\end{bmatrix}
```

and the vertical unit vector:

```math
\mathbf{k} =
\begin{bmatrix}
0\\
0\\
1
\end{bmatrix}
```

The elbow position is

```math
\mathbf{p}_1 =
L_1\cos(q_1)\mathbf{u}
+
L_1\sin(q_1)\mathbf{k}
```

The end-effector position is

```math
\mathbf{p}_{ee} =
\mathbf{p}_1
+
L_2\cos(q_1+q_2)\mathbf{u}
+
L_2\sin(q_1+q_2)\mathbf{k}
```

Forward kinematics provides the joint and end-effector positions used to draw the robot in the 3D scene.

---

## Workspace and Reachability

A target is geometrically reachable when its distance from the base lies inside the two-link workspace shell:

```math
|L_1-L_2|
\le
\sqrt{x^2+y^2+z^2}
\le
L_1+L_2
```

The current implementations also restrict targets to the upper workspace:

```math
z \ge 0
```

START and GOAL targets are validated before the pick-and-place cycle is executed.

---

# Pick-and-Place Process

The simulation uses a finite-state sequence similar to:

```text
HOME
  ↓
MOVE TO START
  ↓
PICK
  ↓
MOVE TO GOAL
  ↓
PLACE
  ↓
RETURN HOME
  ↓
WAIT
  └──────── repeat
```

During the PICK state, the displayed object is attached to the end effector. During the PLACE state, it is detached and remains at the goal position.

This attachment is a visualization state rather than a contact-physics or grasp-force simulation.

---

## Cartesian Trajectory Generation

Motion between two Cartesian points is generated with linear interpolation.

For start point `a`, destination `b`, and normalized trajectory parameter `α`:

```math
\mathbf{p}(\alpha)
=
\mathbf{a}
+
\alpha(\mathbf{b}-\mathbf{a})
```

where

```math
\alpha =
\mathrm{clamp}
\left(
\frac{t}{T},0,1
\right)
```

At each frame:

1. the trajectory generator calculates the current Cartesian target;
2. inverse kinematics converts that target into joint angles;
3. forward kinematics calculates the joint positions;
4. raylib renders the robot and the current simulation state.

---

# Shared Software Concepts

Although each implementation follows the conventions of its language, the projects demonstrate the same general software architecture:

- **Robot model** — link parameters, joint variables, forward kinematics, and inverse kinematics;
- **Trajectory module** — time-based Cartesian interpolation;
- **Simulation state machine** — HOME, PICK, PLACE, RETURN, and WAIT phases;
- **Rendering module** — robot links, joints, base, coordinate axes, object, and end-effector tool;
- **User-interface module** — simulation status, robot parameters, reachability information, and controls;
- **Application loop** — input handling, time-step update, kinematic calculation, and frame rendering.

This structure allows readers to compare modular design, type systems, memory models, object-oriented and functional styles, dependency management, and foreign-function bindings while keeping the robotics problem constant.

---

# raylib Visualization

All projects use raylib or a raylib binding to provide:

- a real-time 3D rendering loop;
- perspective-camera visualization;
- robot-base, joint, link, and tool geometry;
- coordinate axes and workspace context;
- a movable pick-and-place object;
- status and parameter overlays;
- mouse and keyboard interaction.

The exact raylib wrapper and API syntax differ between languages.

---

# Educational Goals

This repository can be used to study:

- forward and inverse kinematics;
- closed-form IK for a yaw-plus-planar RRR arm;
- workspace and reachability analysis;
- elbow-up and elbow-down IK branches;
- Cartesian trajectory interpolation;
- finite-state-machine design;
- real-time simulation loops;
- 3D graphics with raylib;
- cross-language software architecture;
- package managers and build systems;
- numerical safety and floating-point behavior.

It is also useful for developers who want to compare how the same mathematical algorithm is represented in statically typed, dynamically typed, compiled, interpreted, object-oriented, and multi-paradigm languages.

---

# Scope and Limitations

This is primarily a **kinematic visualization project**.

It currently does not model:

- actuator torques;
- joint acceleration dynamics;
- gravity, Coriolis, or centrifugal effects;
- rigid-body contact forces;
- collision detection and avoidance;
- gripper force or suction physics;
- trajectory optimization;
- velocity or acceleration continuity;
- singularity-robust numerical IK;
- feedback control of a physical robot.

Linear Cartesian interpolation can also produce nonlinear joint motion and large joint velocities near singular configurations. These limitations make the project suitable for learning and visualization, but not a complete industrial robot simulator.

---

# Comparing the Implementations

A useful comparison workflow is:

1. run one implementation and record the selected START and GOAL points;
2. use the same points in another implementation;
3. compare the rendered path and pick-and-place sequence;
4. inspect how each language represents vectors, robot state, modules, and dependencies;
5. compare the implementation of IK, FK, trajectory interpolation, and the application loop.

Small differences in numerical output may occur because of floating-point types, graphics bindings, frame timing, and implementation-specific details.

---

# Adding Another Language

A new implementation should be placed in a self-contained folder such as:

```text
Manipulator3D_<Language>/
```

It should preferably preserve the shared conceptual contract:

- three revolute joints;
- the same coordinate and angle conventions;
- analytic IK;
- FK-based rendering;
- reachability validation;
- linear Cartesian motion;
- the same pick-and-place state sequence;
- raylib-based visualization;
- an implementation-specific README with installation and run instructions.

Avoid making one language project depend on source files or generated assets inside another language folder.

---

# Notes for Contributors

When modifying an implementation:

- preserve its independent build process;
- use paths relative to that implementation's folder;
- do not commit generated build output;
- document new dependencies in the child README;
- keep the robotics conventions consistent across languages;
- clearly document intentional behavioral differences.

Typical generated folders that should remain ignored include `build/`, `target/`, `bin/`, `obj/`, `.gradle/`, `.idea/`, `.venv/`, `__pycache__/`, and language-specific package caches.

---

## Acknowledgment

This repository demonstrates how one robotics simulation can be expressed through multiple programming ecosystems while preserving the underlying mathematics and behavior.

The goal is not to identify a single “best” language, but to make the trade-offs, syntax, tooling, and implementation styles directly comparable.
