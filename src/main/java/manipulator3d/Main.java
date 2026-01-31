package manipulator3d;

import manipulator3d.robot.*;
import manipulator3d.sim.LinearTrajectory;
import manipulator3d.ui.Overlay;
import manipulator3d.render.DrawUtils;

import static com.raylib.Raylib.*;
import static com.raylib.Colors.*;
import com.raylib.Raylib.*;

import java.io.BufferedReader;
import java.io.InputStreamReader;

/**
 * 3-DOF two-link manipulator simulation:
 * - Prompts for START and GOAL points
 * - Runs a pick-and-place loop with a fixed HOME end-effector position (2,2,2)
 * - Uses analytic IK to command joint angles each frame
 */
public final class Main {

    private enum Phase {
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

    private static boolean tryParseVec3(String line, Vec3f out) {
        if (line == null) return false;
        String[] parts = line.trim().split("\\s+");
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

    private static Vec3f promptVec3(BufferedReader br, String label, Vec3f def) {
        while (true) {
            System.out.printf("%s  (format: x y z, z>=0; Enter=default %.3f %.3f %.3f): ",
                    label, def.x, def.y, def.z);

            String line;
            try {
                line = br.readLine();
            } catch (Exception e) {
                return new Vec3f(def.x, def.y, def.z);
            }

            if (line == null || line.trim().isEmpty()) {
                return new Vec3f(def.x, def.y, def.z);
            }

            Vec3f v = new Vec3f();
            if (!tryParseVec3(line, v)) {
                System.out.println("Invalid format. Example:  1 2 1");
                continue;
            }
            if (v.z < 0.0f) {
                System.out.println("Invalid input: z must be >= 0");
                continue;
            }
            return v;
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
                Font f = LoadFontEx(path, px, null, 0);
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

        // Defaults
        Vec3f start = new Vec3f(1, 2, 1);
        Vec3f goal  = new Vec3f(2, 3, 2);

        // Fixed HOME end-effector position
        final Vec3f homeEE = new Vec3f(2.0f, 2.0f, 2.0f);

        System.out.println("Manipulator3D IK Pick&Place");
        System.out.println("INPUT FORMAT: x y z  (three numbers separated by spaces)");
        System.out.println("Example:  1 2 1");
        System.out.println("Axis rule: z must be >= 0 ; x and y can be negative.");
        System.out.printf ("Workspace: |p| in [%.3f, %.3f] meters%n", arm.minReach(), arm.maxReach());
        System.out.println("Fixed EE HOME position: (2 2 2)
");

        BufferedReader br = new BufferedReader(new InputStreamReader(System.in));
        start = promptVec3(br, "Enter START", start);
        goal  = promptVec3(br, "Enter GOAL ", goal);

        // Validate endpoints + home
        IKResult ikHome  = arm.solveIK(homeEE, false);
        IKResult ikStart = arm.solveIK(start,  false);
        IKResult ikGoal  = arm.solveIK(goal,   false);

        System.out.println("\nUsing:");
        System.out.printf ("  HOME  = (%.3f, %.3f, %.3f) -> %s%n",
                homeEE.x, homeEE.y, homeEE.z, ikHome.reachable ? "reachable" : "NOT reachable");
        if (!ikHome.reachable) System.out.println("    reason: " + ikHome.message);

        System.out.printf ("  START = (%.3f, %.3f, %.3f) -> %s%n",
                start.x, start.y, start.z, ikStart.reachable ? "reachable" : "NOT reachable");
        if (!ikStart.reachable) System.out.println("    reason: " + ikStart.message);

        System.out.printf ("  GOAL  = (%.3f, %.3f, %.3f) -> %s%n",
                goal.x, goal.y, goal.z, ikGoal.reachable ? "reachable" : "NOT reachable");
        if (!ikGoal.reachable) System.out.println("    reason: " + ikGoal.message);
        System.out.println();

        InitWindow(1280, 720, "Manipulator3D - 3DOF IK Pick&Place");
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

        boolean paused = false;

        // Timing
        final float moveHomeToStart = 2.2f;
        final float pickDuration    = 0.45f;
        final float moveStartToGoal = 2.6f;
        final float placeDuration   = 0.35f;
        final float returnToHome    = 2.0f;
        final float resetWaitTotal  = 1.5f;

        float timer = 0.0f;

        // Ball size
        final float ballRadius = clamp(0.03f * reach, 0.06f, 0.16f);

        BallState ballState = BallState.AtStart;
        Vec3f ballPos = new Vec3f(start.x, start.y, start.z);

        Vec3f targetEE = new Vec3f(homeEE.x, homeEE.y, homeEE.z);
        JointAngles qcmd = new JointAngles();

        LinearTrajectory traj = new LinearTrajectory();

        Phase phase = Phase.MoveHomeToStart;

        if (!ikHome.reachable || !ikStart.reachable || !ikGoal.reachable) {
            phase = Phase.Error;
        } else {
            qcmd.set(ikHome.q);
            targetEE.set(homeEE);

            ballState = BallState.AtStart;
            ballPos.set(start);

            traj.reset(homeEE, start, moveHomeToStart);
            phase = Phase.MoveHomeToStart;
            timer = 0.0f;
        }

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

        while (!WindowShouldClose()) {
            if (IsKeyPressed(KEY_F11)) ToggleFullscreen();

            int screenW = GetScreenWidth();
            int screenH = GetScreenHeight();

            updateZoom(cam);

            float dt = paused ? 0.0f : GetFrameTime();
            String runtimeError = null;

            if (phase != Phase.Error) {
                switch (phase) {
                    case MoveHomeToStart -> {
                        ballState = BallState.AtStart;
                        ballPos.set(start);

                        traj.update(dt);
                        targetEE = traj.position();

                        if (traj.finished()) {
                            phase = Phase.PickAtStart;
                            timer = 0.0f;
                            targetEE.set(start);
                        }
                    }
                    case PickAtStart -> {
                        targetEE.set(start);
                        ballState = BallState.AtStart;
                        ballPos.set(start);

                        timer += dt;
                        if (timer >= pickDuration) {
                            ballState = BallState.Attached;
                            timer = 0.0f;
                            traj.reset(start, goal, moveStartToGoal);
                            phase = Phase.MoveStartToGoal;
                        }
                    }
                    case MoveStartToGoal -> {
                        traj.update(dt);
                        targetEE = traj.position();
                        ballState = BallState.Attached;

                        if (traj.finished()) {
                            phase = Phase.PlaceAtGoal;
                            timer = 0.0f;
                            targetEE.set(goal);
                        }
                    }
                    case PlaceAtGoal -> {
                        targetEE.set(goal);

                        timer += dt;
                        if (timer >= placeDuration) {
                            ballState = BallState.AtGoal;
                            timer = 0.0f;
                            traj.reset(goal, homeEE, returnToHome);
                            phase = Phase.ReturnGoalToHome;
                        } else {
                            ballState = BallState.Attached;
                        }
                    }
                    case ReturnGoalToHome -> {
                        ballState = BallState.AtGoal;

                        traj.update(dt);
                        targetEE = traj.position();

                        timer += dt;
                        if (traj.finished()) {
                            phase = Phase.WaitAtHomeReset;
                        }
                    }
                    case WaitAtHomeReset -> {
                        targetEE.set(homeEE);

                        timer += dt;
                        if (timer >= resetWaitTotal) {
                            ballState = BallState.AtStart;
                            ballPos.set(start);

                            timer = 0.0f;
                            traj.reset(homeEE, start, moveHomeToStart);
                            phase = Phase.MoveHomeToStart;
                        } else {
                            ballState = BallState.AtGoal;
                        }
                    }
                    default -> {}
                }

                IKResult ikNow = arm.solveIK(targetEE, false);
                if (!ikNow.reachable) {
                    phase = Phase.Error;
                    runtimeError = ikNow.message;
                } else {
                    qcmd.set(ikNow.q);
                }
            }

            // FK for rendering and tool approach direction
            FKResult fk = arm.forwardKinematics(qcmd);

            Vec3f approach = Vec3f.sub(fk.ee, fk.joint2);
            approach = Vec3f.normalize(approach);

            // Ball position by state (always visible)
            if (ballState == BallState.AtStart) {
                ballPos.set(start);
            } else if (ballState == BallState.AtGoal) {
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

            // Ball (smaller + outline)
            DrawSphere(vBall, ballRadius, RED);
            DrawSphereWires(vBall, ballRadius * 1.02f, 10, 10, RAYWHITE);

            EndMode3D();

            Overlay.OverlayStatus st = new Overlay.OverlayStatus();
            st.startReachable = ikStart.reachable;
            st.endReachable   = ikGoal.reachable;
            st.errorText      = runtimeError;

            st.phaseText = switch (phase) {
                case MoveHomeToStart -> "Phase: HOME -> START";
                case PickAtStart     -> "Phase: PICK at START";
                case MoveStartToGoal -> "Phase: START -> GOAL (ball attached)";
                case PlaceAtGoal     -> "Phase: PLACE at GOAL";
                case ReturnGoalToHome-> "Phase: GOAL -> HOME";
                case WaitAtHomeReset -> "Phase: WAIT then LOOP";
                case Error           -> "Phase: ERROR";
            };

            if (phase == Phase.Error && runtimeError == null) {
                st.errorText = "ERROR: HOME/START/GOAL invalid or out of reach (z>=0 required).";
            }

            paused = Overlay.drawOverlayPanel(uiFont, arm, start, goal, st, paused, screenW, screenH);

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
