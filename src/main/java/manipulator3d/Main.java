package manipulator3d;

import manipulator3d.robot.*;
import manipulator3d.sim.LinearTrajectory;
import manipulator3d.ui.Overlay;
import manipulator3d.render.DrawUtils;

import static com.raylib.Raylib.*;
import static com.raylib.Colors.*;
import com.raylib.Raylib.*;

import org.bytedeco.javacpp.IntPointer;

/**
 * 3-DOF two-link manipulator simulation:
 * - Start/Goal inputs are entered in the UI panel while paused.
 * - Press PLAY to validate/apply inputs and start the pick-and-place loop.
 * - Press PAUSE to freeze; edit points; press PLAY to restart from HOME.
 *
 * Speed control:
 * - SIM_SPEED = 1.75f increases motion speed by ~75%.
 */
public final class Main {

    // Increase motion speed by ~75%
    private static final float SIM_SPEED = 1.75f;

    private enum Phase {
        Idle,               // waiting for PLAY
        MoveHomeToStart,
        PickAtStart,
        MoveStartToGoal,
        PlaceAtGoal,
        ReturnGoalToHome,
        WaitAtHomeReset,
        Error
    }

    private enum BallState {
        AtStart,
        Attached,
        AtGoal
    }

    private static float clamp(float v, float lo, float hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    private static boolean tryParseVec3Text(String text, Vec3f out) {
        if (text == null) return false;
        String t = text.trim();
        if (t.isEmpty()) return false;
        String[] parts = t.split("\\s+");
        if (parts.length != 3) return false;
        try {
            float x = Float.parseFloat(parts[0]);
            float y = Float.parseFloat(parts[1]);
            float z = Float.parseFloat(parts[2]);
            out.set(x, y, z);
            return true;
        } catch (NumberFormatException ex) {
            return false;
        }
    }

    private static Font loadBestUIFont(int px) {
        String[] candidates = new String[] {
                "resources/fonts/Inter-Regular.ttf",
                "../resources/fonts/Inter-Regular.ttf",
                "C:/Windows/Fonts/segoeui.ttf",
                "C:/Windows/Fonts/arial.ttf"
        };

        for (String path : candidates) {
            if (FileExists(path)) {
                // Cast null to select the intended overload and avoid ambiguity
                Font f = LoadFontEx(path, px, (IntPointer) null, 0);
                if (f.texture().id() != 0) {
                    SetTextureFilter(f.texture(), TEXTURE_FILTER_BILINEAR);
                    return f;
                }
            }
        }

        Font def = GetFontDefault();
        SetTextureFilter(def.texture(), TEXTURE_FILTER_BILINEAR);
        return def;
    }

    private static void updateZoom(Camera3D cam) {
        float wheel = GetMouseWheelMove();
        if (wheel == 0.0f) return;

        Vector3 pos = cam._position();
        Vector3 tgt = cam.target();

        float vx = pos.x() - tgt.x();
        float vy = pos.y() - tgt.y();
        float vz = pos.z() - tgt.z();

        float len = (float)Math.sqrt(vx*vx + vy*vy + vz*vz);
        if (len < 1e-6f) return;

        float scale = 1.0f - wheel * 0.10f;
        scale = clamp(scale, 0.70f, 1.30f);

        float dist = len * scale;
        dist = clamp(dist, 1.0f, 200.0f);

        float inv = 1.0f / len;
        float dx = vx * inv;
        float dy = vy * inv;
        float dz = vz * inv;

        pos.x(tgt.x() + dx * dist);
        pos.y(tgt.y() + dy * dist);
        pos.z(tgt.z() + dz * dist);
    }

    private static void setVec(Vector3 dst, Vec3f src) {
        dst.x(src.x).y(src.y).z(src.z);
    }

    private static Vector3 v(float x, float y, float z) {
        return new Vector3().x(x).y(y).z(z);
    }

    private static void restartFromHome(
            RobotArm arm,
            Vec3f homeEE,
            Vec3f start,
            Vec3f goal,
            JointAngles qcmd,
            LinearTrajectory traj,
            float moveHomeToStart,
            Holder<Phase> phase,
            Holder<Float> timer,
            Holder<BallState> ballState,
            Vec3f targetEE,
            Holder<String> runtimeError
    ) {
        runtimeError.value = null;

        IKResult ikHome  = arm.solveIK(homeEE, false);
        IKResult ikStart = arm.solveIK(start,  false);
        IKResult ikGoal  = arm.solveIK(goal,   false);

        if (!ikHome.reachable || !ikStart.reachable || !ikGoal.reachable) {
            phase.value = Phase.Error;
            if (!ikHome.reachable) runtimeError.value = ikHome.message;
            else if (!ikStart.reachable) runtimeError.value = ikStart.message;
            else runtimeError.value = ikGoal.message;
            return;
        }

        // Arm starts at HOME pose
        qcmd.set(ikHome.q);
        targetEE.set(homeEE);

        // Ball starts at START
        ballState.value = BallState.AtStart;

        // Begin: HOME -> START
        traj.reset(homeEE, start, moveHomeToStart);
        phase.value = Phase.MoveHomeToStart;
        timer.value = 0.0f;
    }

    /** Minimal mutable holder to avoid lots of single-element arrays. */
    private static final class Holder<T> { T value; Holder(T v){ value = v; } }

    public static void main(String[] args) throws Exception {
        SetTraceLogLevel(LOG_ERROR);
        SetConfigFlags(FLAG_WINDOW_RESIZABLE | FLAG_MSAA_4X_HINT);

        // Arm parameters
        LinkParams link1 = new LinkParams();
        link1.lengthM = 3.0f;
        link1.massKg  = 2.0f;
        link1.recomputeInertia();

        LinkParams link2 = new LinkParams();
        link2.lengthM = 2.6f;
        link2.massKg  = 1.6f;
        link2.recomputeInertia();

        RobotArm arm = new RobotArm(link1, link2);

        // Active targets (used by simulation)
        Vec3f start = new Vec3f(1, 2, 1);
        Vec3f goal  = new Vec3f(2, 3, 2);

        // Fixed HOME end-effector position
        final Vec3f homeEE = new Vec3f(2.0f, 2.0f, 2.0f);

        System.out.println("Manipulator3D IK Pick&Place");
        System.out.println("Inputs are entered in the UI panel while paused.");
        System.out.println("Axis rule: z must be >= 0 ; x and y can be negative.");
        System.out.printf ("Workspace: |p| in [%.3f, %.3f] meters%n", arm.minReach(), arm.maxReach());
        System.out.println("Fixed EE HOME position: (2 2 2)\n");

        InitWindow(1280, 720, "Manipulator3D - 3DOF IK Pick&Place (Java)");
        SetWindowMinSize(960, 540);
        SetTargetFPS(60);

        Font uiFont = loadBestUIFont(22);

        // Camera framing
        float reach = arm.maxReach();
        Camera3D cam = new Camera3D()
                .target(v(0.0f, 0.0f, 0.35f * reach))
                .up(v(0.0f, 0.0f, 1.0f))
                .fovy(52.0f)
                .projection(CAMERA_PERSPECTIVE)
                ._position(v(1.10f * reach, -1.15f * reach, 0.85f * reach));

        // Start paused: user enters points in UI, then presses PLAY
        boolean paused = true;

        // Timing (base values; SIM_SPEED scales the effective dt)
        final float moveHomeToStart = 2.2f;
        final float pickDuration    = 0.45f;
        final float moveStartToGoal = 2.6f;
        final float placeDuration   = 0.35f;
        final float returnToHome    = 2.0f;
        final float resetWaitTotal  = 1.5f;

        Holder<Float> timer = new Holder<>(0.0f);

        // Ball size
        final float ballRadius = clamp(0.03f * reach, 0.06f, 0.16f);

        Holder<BallState> ballState = new Holder<>(BallState.AtStart);
        Vec3f ballPos = new Vec3f(start.x, start.y, start.z);

        Vec3f targetEE = new Vec3f(homeEE.x, homeEE.y, homeEE.z);
        JointAngles qcmd = new JointAngles();

        LinearTrajectory traj = new LinearTrajectory();

        Holder<Phase> phase = new Holder<>(Phase.Idle);
        Holder<String> runtimeError = new Holder<>(null);

        // Initialize arm at HOME pose (idle)
        IKResult ikHomeInit = arm.solveIK(homeEE, false);
        if (ikHomeInit.reachable) qcmd.set(ikHomeInit.q);

        // Overlay state (text inputs persist here)
        Overlay.InputState input = new Overlay.InputState();
        input.setStartDefault(start);
        input.setGoalDefault(goal);

        // Reusable native vectors for drawing
        Vector3 vOrigin = v(0,0,0);
        Vector3 vBase   = v(0,0,0);
        Vector3 vJ2     = v(0,0,0);
        Vector3 vEE     = v(0,0,0);
        Vector3 vBall   = v(0,0,0);
        Vector3 vApproach = v(1,0,0);

        // Thick axes
        final float axisLen = 3.0f;
        final float axisR   = 0.03f;
        Vector3 xAxis = v(axisLen, 0, 0);
        Vector3 yAxis = v(0, axisLen, 0);
        Vector3 zAxis = v(0, 0, axisLen);

        // Preview vectors while paused (parsed from UI fields)
        Vec3f pendingStart = new Vec3f(start.x, start.y, start.z);
        Vec3f pendingGoal  = new Vec3f(goal.x, goal.y, goal.z);

        while (!WindowShouldClose()) {
            if (IsKeyPressed(KEY_F11)) ToggleFullscreen();

            int screenW = GetScreenWidth();
            int screenH = GetScreenHeight();

            updateZoom(cam);

            // Scale simulation time by SIM_SPEED (~75% faster)
            float dtRaw = paused ? 0.0f : GetFrameTime();
            float dtSim = dtRaw * SIM_SPEED;

            // While paused, parse pending inputs and compute reachability for UI feedback
            boolean pendingStartOk = tryParseVec3Text(input.startText.toString(), pendingStart) && pendingStart.z >= 0.0f;
            boolean pendingGoalOk  = tryParseVec3Text(input.goalText.toString(),  pendingGoal)  && pendingGoal.z  >= 0.0f;

            IKResult ikStartPending = pendingStartOk ? arm.solveIK(pendingStart, false) : IKResult.unreachable("Invalid START: use 'x y z' with z>=0");
            IKResult ikGoalPending  = pendingGoalOk  ? arm.solveIK(pendingGoal,  false) : IKResult.unreachable("Invalid GOAL: use 'x y z' with z>=0");

            // Simulation update
            if (phase.value != Phase.Error && !paused) {
                switch (phase.value) {
                    case Idle -> {
                        // Should not happen while running, but safe
                        phase.value = Phase.MoveHomeToStart;
                    }

                    case MoveHomeToStart -> {
                        ballState.value = BallState.AtStart;
                        ballPos.set(start);

                        traj.update(dtSim);
                        targetEE = traj.position();

                        if (traj.finished()) {
                            phase.value = Phase.PickAtStart;
                            timer.value = 0.0f;
                            targetEE.set(start);
                        }
                    }

                    case PickAtStart -> {
                        targetEE.set(start);
                        ballState.value = BallState.AtStart;
                        ballPos.set(start);

                        timer.value += dtSim;
                        if (timer.value >= pickDuration) {
                            ballState.value = BallState.Attached;
                            timer.value = 0.0f;
                            traj.reset(start, goal, moveStartToGoal);
                            phase.value = Phase.MoveStartToGoal;
                        }
                    }

                    case MoveStartToGoal -> {
                        traj.update(dtSim);
                        targetEE = traj.position();
                        ballState.value = BallState.Attached;

                        if (traj.finished()) {
                            phase.value = Phase.PlaceAtGoal;
                            timer.value = 0.0f;
                            targetEE.set(goal);
                        }
                    }

                    case PlaceAtGoal -> {
                        targetEE.set(goal);

                        timer.value += dtSim;
                        if (timer.value >= placeDuration) {
                            ballState.value = BallState.AtGoal;
                            timer.value = 0.0f;
                            traj.reset(goal, homeEE, returnToHome);
                            phase.value = Phase.ReturnGoalToHome;
                        } else {
                            ballState.value = BallState.Attached;
                        }
                    }

                    case ReturnGoalToHome -> {
                        ballState.value = BallState.AtGoal;

                        traj.update(dtSim);
                        targetEE = traj.position();

                        timer.value += dtSim; // time since place
                        if (traj.finished()) {
                            phase.value = Phase.WaitAtHomeReset;
                        }
                    }

                    case WaitAtHomeReset -> {
                        targetEE.set(homeEE);

                        timer.value += dtSim;
                        if (timer.value >= resetWaitTotal) {
                            ballState.value = BallState.AtStart;
                            ballPos.set(start);

                            timer.value = 0.0f;
                            traj.reset(homeEE, start, moveHomeToStart);
                            phase.value = Phase.MoveHomeToStart;
                        } else {
                            ballState.value = BallState.AtGoal;
                        }
                    }

                    default -> {}
                }

                // IK for current EE target
                IKResult ikNow = arm.solveIK(targetEE, false);
                if (!ikNow.reachable) {
                    phase.value = Phase.Error;
                    runtimeError.value = ikNow.message;
                } else {
                    qcmd.set(ikNow.q);
                }
            }

            // FK for rendering + approach direction
            FKResult fk = arm.forwardKinematics(qcmd);

            Vec3f approach = Vec3f.sub(fk.ee, fk.joint2);
            approach = Vec3f.normalize(approach);

            // Ball position by state (always visible)
            if (ballState.value == BallState.AtStart) {
                ballPos.set(start);
            } else if (ballState.value == BallState.AtGoal) {
                ballPos.set(goal);
            } else {
                Vec3f mounted = Vec3f.add(fk.ee, Vec3f.scale(approach, 0.22f));
                ballPos.set(mounted);
            }

            // Update native vectors
            setVec(vBase, fk.base);
            setVec(vJ2, fk.joint2);
            setVec(vEE, fk.ee);
            setVec(vBall, ballPos);
            vApproach.x(approach.x).y(approach.y).z(approach.z);

            // ----- Draw -----
            BeginDrawing();
            ClearBackground(DrawUtils.rgba(10, 12, 16, 255));

            BeginMode3D(cam);

            // No grid, thicker axes using cylinders
            DrawCylinderEx(vOrigin, xAxis, axisR, axisR, 12, RED);
            DrawCylinderEx(vOrigin, yAxis, axisR, axisR, 12, GREEN);
            DrawCylinderEx(vOrigin, zAxis, axisR, axisR, 12, BLUE);

            // Robot visuals
            DrawUtils.drawRobotBasePedestal(vOrigin);
            DrawUtils.drawRobotJointHousing(vBase, 0.30f);
            DrawUtils.drawRobotJointHousing(vJ2, 0.24f);
            DrawUtils.drawRobotJointHousing(vEE, 0.18f);

            DrawUtils.drawTaperedLink(vBase, vJ2, 0.14f, 0.12f, DrawUtils.rgba(185,185,190,255));
            DrawUtils.drawTaperedLink(vJ2, vEE,   0.12f, 0.10f, DrawUtils.rgba(170,170,175,255));

            DrawUtils.drawSuctionTool(vEE, vApproach);

            // Ball (outline)
            DrawSphere(vBall, ballRadius, RED);
            DrawSphereWires(vBall, ballRadius * 1.02f, 10, 10, RAYWHITE);

            EndMode3D();

            // ----- Overlay status -----
            Overlay.OverlayStatus st = new Overlay.OverlayStatus();

            // When paused, show reachability for pending inputs (so user gets feedback before PLAY).
            // When running, show reachability for active inputs.
            if (paused) {
                st.startReachable = ikStartPending.reachable;
                st.endReachable   = ikGoalPending.reachable;
            } else {
                st.startReachable = arm.solveIK(start, false).reachable;
                st.endReachable   = arm.solveIK(goal,  false).reachable;
            }

            String phaseText = switch (phase.value) {
                case Idle            -> "Phase: IDLE (enter inputs, press PLAY)";
                case MoveHomeToStart -> "Phase: HOME -> START";
                case PickAtStart     -> "Phase: PICK at START";
                case MoveStartToGoal -> "Phase: START -> GOAL (ball attached)";
                case PlaceAtGoal     -> "Phase: PLACE at GOAL";
                case ReturnGoalToHome-> "Phase: GOAL -> HOME";
                case WaitAtHomeReset -> "Phase: WAIT then LOOP";
                case Error           -> "Phase: ERROR";
            };
            st.phaseText = paused ? (phaseText + "  [PAUSED]") : phaseText;

            // Error message priority:
            // - runtime errors while running
            // - input parse/reach errors while paused
            if (!paused && phase.value == Phase.Error) {
                st.errorText = (runtimeError.value != null) ? runtimeError.value
                        : "ERROR: HOME/START/GOAL invalid or out of reach (z>=0 required).";
            } else if (paused) {
                // Only show something if there is a real problem
                if (!pendingStartOk) st.errorText = "START format invalid. Use: x y z   (z>=0)";
                else if (!pendingGoalOk) st.errorText = "GOAL format invalid. Use: x y z   (z>=0)";
                else if (!ikStartPending.reachable) st.errorText = "START not reachable: " + ikStartPending.message;
                else if (!ikGoalPending.reachable) st.errorText = "GOAL not reachable: " + ikGoalPending.message;
                else st.errorText = null;
            } else {
                st.errorText = null;
            }

            // Overlay returns an action event (Main applies it)
            Overlay.Action action = Overlay.drawOverlayPanel(
                    uiFont, arm, start, goal, input, st, paused, screenW, screenH
            );

            if (action == Overlay.Action.PAUSE_PRESSED) {
                paused = true;
                phase.value = Phase.Idle; // freeze and wait for new PLAY
                runtimeError.value = null;

                // Snap arm back to HOME pose visually while paused (consistent "start from home")
                IKResult ikHome = arm.solveIK(homeEE, false);
                if (ikHome.reachable) {
                    qcmd.set(ikHome.q);
                    targetEE.set(homeEE);
                }
                ballState.value = BallState.AtStart;
                ballPos.set(start);
            }

            if (action == Overlay.Action.PLAY_PRESSED) {
                // Validate pending inputs and apply
                if (pendingStartOk && pendingGoalOk && ikStartPending.reachable && ikGoalPending.reachable) {
                    // Apply new active targets
                    start.set(pendingStart);
                    goal.set(pendingGoal);

                    // Restart sim from HOME using active targets
                    restartFromHome(
                            arm, homeEE, start, goal, qcmd, traj, moveHomeToStart,
                            phase, timer, ballState, targetEE, runtimeError
                    );

                    // Only unpause if restart succeeded
                    if (phase.value != Phase.Error) {
                        paused = false;
                    } else {
                        paused = true;
                    }
                } else {
                    // Keep paused; st.errorText already explains why
                    paused = true;
                }
            }

            DrawUtils.drawTextSmall(uiFont, "F11: fullscreen   Mouse Wheel: zoom", 12, screenH - 28, 18,
                    DrawUtils.rgba(200,200,200,220));

            EndDrawing();
        }

        if (uiFont.texture().id() != GetFontDefault().texture().id()) {
            UnloadFont(uiFont);
        }
        CloseWindow();
    }
}
