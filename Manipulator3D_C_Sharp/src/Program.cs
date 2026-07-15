using System;
using System.Globalization;
using System.Numerics;
using Raylib_cs;
using System.IO;
using Manipulator3D.robot;
using Manipulator3D.sim;
using Manipulator3D.ui;
using Manipulator3D.render;

namespace Manipulator3D;

internal static class Program
{
    private static float Clamp(float v, float lo, float hi) => MathF.Max(lo, MathF.Min(hi, v));

private static Font LoadBestUIFont(int px)
{
    string baseDir = AppContext.BaseDirectory;

    string[] candidates =
    {
        Path.Combine(baseDir, "resources", "fonts", "Inter-Regular.ttf"),
        Path.Combine(baseDir, "..", "..", "..", "resources", "fonts", "Inter-Regular.ttf"), // dev fallback
        Path.Combine(Environment.GetFolderPath(Environment.SpecialFolder.Windows), "Fonts", "segoeui.ttf"),
        Path.Combine(Environment.GetFolderPath(Environment.SpecialFolder.Windows), "Fonts", "arial.ttf"),
    };

    foreach (var path in candidates)
    {
        if (!File.Exists(path)) continue;

        Font f = Raylib.LoadFontEx(path, px, null, 0);

        // Optional: smoother font sampling
        Raylib.SetTextureFilter(f.Texture, RaylibCompat.TextureBilinear);

        return f; // If file exists, this is enough; no texture.id check needed
    }

    Font def = Raylib.GetFontDefault();
    Raylib.SetTextureFilter(def.Texture, RaylibCompat.TextureBilinear);
    return def;
}


private static void UpdateZoom(ref Vector3 camPosition, Vector3 camTarget)
    {
        float wheel = Raylib.GetMouseWheelMove();
        if (MathF.Abs(wheel) < 1e-6f) return;

        Vector3 v = camPosition - camTarget;
        float dist = v.Length();
        if (dist < 0.001f) dist = 0.001f;

        float scale = 1.0f - wheel * 0.10f;
        scale = Clamp(scale, 0.70f, 1.30f);

        dist *= scale;
        dist = Clamp(dist, 1.0f, 200.0f);

        Vector3 dir = dist > 1e-6f ? Vector3.Normalize(v) : Vector3.UnitX;
        camPosition = camTarget + dir * dist;
    }

    private enum Phase
    {
        MoveHomeToStart,
        PickAtStart,
        MoveStartToGoal,
        PlaceAtGoal,
        ReturnGoalToHome,
        WaitAtHomeReset,
        Error
    }

    private enum BallState
    {
        AtStart,
        Attached,
        AtGoal
    }

    [STAThread]
    public static void Main()
    {
        // IMPORTANT: config flags must be set before InitWindow
        Raylib.SetTraceLogLevel(RaylibCompat.TraceLogError);
        Raylib.SetConfigFlags(RaylibCompat.WindowResizable | RaylibCompat.Msaa4xHint);

        // --- Arm parameters ---
        LinkParams link1 = new()
        {
            length_m = 3.0f,
            mass_kg = 2.0f
        };
        link1.RecomputeInertia();

        LinkParams link2 = new()
        {
            length_m = 2.6f,
            mass_kg = 1.6f
        };
        link2.RecomputeInertia();

        RobotArm arm = new(link1, link2);

        // Defaults (also used to seed the UI fields)
        Vector3 start = new(1f, 2f, 1f);
        Vector3 goal = new(2f, 3f, 2f);

        // Fixed end-effector "home" position
        Vector3 homeEE = new(2f, 2f, 2f);

        // Window + timing
        Raylib.InitWindow(1280, 720, "Manipulator3D - 3DOF IK Pick&Place (C#)");
        Raylib.SetWindowMinSize(960, 540);
        Raylib.SetTargetFPS(60);

        Font uiFont = LoadBestUIFont(22);

        // Camera framing
        float reach = arm.MaxReach();
        Vector3 camTarget = new(0.0f, 0.0f, 0.35f * reach);
        Vector3 camUp = Vector3.UnitZ;
        float camFovy = 52.0f;
        Vector3 camPosition = new(1.10f * reach, -1.15f * reach, 0.85f * reach);

        // UI / interaction state
        bool paused = true; // start paused so the user can set START/GOAL in the overlay
        OverlayUiState overlayUi = new(start, goal);

        // Timing
        const float moveHomeToStart = 2.2f;
        const float pickDuration = 0.45f;
        const float moveStartToGoal = 2.6f;
        const float placeDuration = 0.35f;
        const float returnToHome = 2.0f;
        const float resetWaitTotal = 1.5f;

        float timer = 0.0f;

        // Ball size (scaled to arm reach)
        float ballRadius = Clamp(0.03f * reach, 0.06f, 0.16f);

        BallState ballState = BallState.AtStart;
        Vector3 ballPos = start;

        Vector3 targetEE = homeEE;
        JointAngles qcmd = new();

        LinearTrajectory traj = new();

        Phase phase = Phase.Error;
        string? runtimeError = null;

        // Helper: (re)start the full pick-and-place loop from HOME -> START.
        bool TryStartNewCycle(Vector3 newStart, Vector3 newGoal, out string? error)
        {
            error = null;

            IKResult ikHome = arm.SolveIK(homeEE, elbowUp: false);
            IKResult ikStart = arm.SolveIK(newStart, elbowUp: false);
            IKResult ikGoal = arm.SolveIK(newGoal, elbowUp: false);

            if (!ikHome.reachable)
            {
                error = $"HOME not reachable: {ikHome.message}";
                return false;
            }

            if (!ikStart.reachable)
            {
                error = $"START not reachable: {ikStart.message}";
                return false;
            }

            if (!ikGoal.reachable)
            {
                error = $"GOAL not reachable: {ikGoal.message}";
                return false;
            }

            // Commit the new endpoints to the simulator state
            start = newStart;
            goal = newGoal;

            // Reset arm + ball to initial loop conditions
            qcmd = ikHome.q;
            targetEE = homeEE;

            ballState = BallState.AtStart;
            ballPos = start;

            traj.Reset(homeEE, start, moveHomeToStart);
            phase = Phase.MoveHomeToStart;

            timer = 0.0f;
            runtimeError = null;

            return true;
        }

        // Main loop
        while (!Raylib.WindowShouldClose())
        {
            if (Raylib.IsKeyPressed(RaylibCompat.KeyF11))
                Raylib.ToggleFullscreen();

            int screenW = Raylib.GetScreenWidth();
            int screenH = Raylib.GetScreenHeight();

            UpdateZoom(ref camPosition, camTarget);

            float dt = paused ? 0.0f : Raylib.GetFrameTime();

            // Parse "pending" endpoints from UI text (used for reachability feedback even before applying).
            Vector3 pendingStart = start;
            Vector3 pendingGoal = goal;

            string? inputError = null;
            bool havePending = overlayUi.TryGetPendingTargets(out pendingStart, out pendingGoal, out inputError);

            IKResult ikStartNow = new() { reachable = false, message = "" };
            IKResult ikGoalNow = new() { reachable = false, message = "" };

            if (havePending)
            {
                ikStartNow = arm.SolveIK(pendingStart, elbowUp: false);
                ikGoalNow = arm.SolveIK(pendingGoal, elbowUp: false);
            }

            // Update finite state machine
            if (!paused && phase != Phase.Error)
            {
                switch (phase)
                {
                    case Phase.MoveHomeToStart:
                        {
                            ballState = BallState.AtStart;
                            ballPos = start;

                            traj.Update(dt);
                            targetEE = traj.Position();

                            if (traj.Finished())
                            {
                                phase = Phase.PickAtStart;
                                timer = 0.0f;
                                targetEE = start;
                            }
                        }
                        break;

                    case Phase.PickAtStart:
                        {
                            targetEE = start;
                            ballState = BallState.AtStart;
                            ballPos = start;

                            timer += dt;
                            if (timer >= pickDuration)
                            {
                                ballState = BallState.Attached;
                                timer = 0.0f;
                                traj.Reset(start, goal, moveStartToGoal);
                                phase = Phase.MoveStartToGoal;
                            }
                        }
                        break;

                    case Phase.MoveStartToGoal:
                        {
                            traj.Update(dt);
                            targetEE = traj.Position();
                            ballState = BallState.Attached;

                            if (traj.Finished())
                            {
                                phase = Phase.PlaceAtGoal;
                                timer = 0.0f;
                                targetEE = goal;
                            }
                        }
                        break;

                    case Phase.PlaceAtGoal:
                        {
                            targetEE = goal;

                            timer += dt;
                            if (timer >= placeDuration)
                            {
                                ballState = BallState.AtGoal;
                                timer = 0.0f; // reuse as "time since place"
                                traj.Reset(goal, homeEE, returnToHome);
                                phase = Phase.ReturnGoalToHome;
                            }
                            else
                            {
                                ballState = BallState.Attached;
                            }
                        }
                        break;

                    case Phase.ReturnGoalToHome:
                        {
                            ballState = BallState.AtGoal;

                            traj.Update(dt);
                            targetEE = traj.Position();

                            timer += dt; // time since place
                            if (traj.Finished())
                            {
                                phase = Phase.WaitAtHomeReset;
                            }
                        }
                        break;

                    case Phase.WaitAtHomeReset:
                        {
                            targetEE = homeEE;

                            timer += dt;
                            if (timer >= resetWaitTotal)
                            {
                                ballState = BallState.AtStart;
                                ballPos = start;

                                timer = 0.0f;
                                traj.Reset(homeEE, start, moveHomeToStart);
                                phase = Phase.MoveHomeToStart;
                            }
                            else
                            {
                                ballState = BallState.AtGoal;
                            }
                        }
                        break;
                }

                // Solve IK for the current end-effector target
                IKResult ikNow = arm.SolveIK(targetEE, elbowUp: false);
                if (!ikNow.reachable)
                {
                    phase = Phase.Error;
                    runtimeError = ikNow.message;
                }
                else
                {
                    qcmd = ikNow.q;
                }
            }

            // FK for rendering + ball attachment offset
            FKResult fk = arm.ForwardKinematics(qcmd);

            // Tool approach direction: from elbow to EE
            Vector3 approach = fk.ee - fk.joint2;
            float alen = approach.Length();
            approach = alen > 1e-6f ? approach / alen : Vector3.UnitX;

            // Ball position by state (always visible)
            if (ballState == BallState.AtStart)
                ballPos = start;
            else if (ballState == BallState.AtGoal)
                ballPos = goal;
            else
                ballPos = fk.ee + approach * 0.22f;

            // Overlay status (reachability feedback is based on pending inputs while paused)
            OverlayStatus st = new()
            {
                startReachable = havePending && ikStartNow.reachable,
                endReachable = havePending && ikGoalNow.reachable,
                errorText = null,
                phaseText = PhaseText(phase)
            };

            if (!string.IsNullOrWhiteSpace(inputError))
                st.errorText = inputError;

            if (phase == Phase.Error)
            {
                st.errorText ??= runtimeError ?? "ERROR: HOME/START/GOAL invalid or\nout of reach (z>=0 required).";
            }

            // Draw
            Raylib.BeginDrawing();
            Raylib.ClearBackground(Raylib.GetColor(0x0A0C10FF)); // 10,12,16,255

            Camera3D cam = new()
            {
                Position = camPosition,
                Target = camTarget,
                Up = camUp,
                FovY = camFovy,
                Projection = RaylibCompat.CameraPerspective
            };

            Raylib.BeginMode3D(cam);

            // Thicker axes using cylinders
            const float axisLen = 3.0f;
            const float axisR = 0.03f;

            Raylib.DrawCylinderEx(Vector3.Zero, new Vector3(axisLen, 0, 0), axisR, axisR, 12, Color.Red);
            Raylib.DrawCylinderEx(Vector3.Zero, new Vector3(0, axisLen, 0), axisR, axisR, 12, Color.Green);
            Raylib.DrawCylinderEx(Vector3.Zero, new Vector3(0, 0, axisLen), axisR, axisR, 12, Color.Blue);

            // Robot visuals
            DrawUtils.DrawRobotBasePedestal(Vector3.Zero);
            DrawUtils.DrawRobotJointHousing(fk.basePos, 0.30f);
            DrawUtils.DrawRobotJointHousing(fk.joint2, 0.24f);
            DrawUtils.DrawRobotJointHousing(fk.ee, 0.18f);

            DrawUtils.DrawTaperedLink(fk.basePos, fk.joint2, 0.14f, 0.12f, Raylib.GetColor(0xB9B9BEFF));
            DrawUtils.DrawTaperedLink(fk.joint2, fk.ee, 0.12f, 0.10f, Raylib.GetColor(0xAAAAAFFF));

            DrawUtils.DrawSuctionTool(fk.ee, approach);

            // Ball + outline
            Raylib.DrawSphere(ballPos, ballRadius, Color.Red);
            Raylib.DrawSphereWires(ballPos, ballRadius * 1.02f, 10, 10, Color.RayWhite);

            Raylib.EndMode3D();

            // Overlay (inputs + status)
            OverlayResult uiResult = Overlay.DrawOverlayPanel(
                font: uiFont,
                arm: arm,
                acceptedStart: start,
                acceptedGoal: goal,
                pendingStart: havePending ? pendingStart : (Vector3?)null,
                pendingGoal: havePending ? pendingGoal : (Vector3?)null,
                status: st,
                paused: paused,
                uiState: overlayUi,
                screenW: screenW,
                screenH: screenH
            );

            // Apply interactions
            if (!paused && uiResult.pauseClicked)
            {
                paused = true;
            }
            else if (paused && uiResult.playClicked)
            {
                // Attempt to start a new cycle using the current text fields
                if (!overlayUi.TryGetPendingTargets(out Vector3 newStart, out Vector3 newGoal, out string? err))
                {
                    runtimeError = err ?? "Invalid START/GOAL input.";
                    phase = Phase.Error;
                    paused = true;
                }
                else
                {
                    if (TryStartNewCycle(newStart, newGoal, out string? err2))
                    {
                        // Keep the text fields in sync with the applied values
                        overlayUi.SyncFromAccepted(start, goal);
                        paused = false;
                    }
                    else
                    {
                        runtimeError = err2;
                        phase = Phase.Error;
                        paused = true;
                    }
                }
            }

            // HUD hint
            DrawUtils.DrawTextSmall(uiFont, "F11: fullscreen   Mouse Wheel: zoom", 12, screenH - 28, 18, Raylib.GetColor(0xC8C8C8DC));

            Raylib.EndDrawing();
        }

        Raylib.CloseWindow();
    }

    private static string PhaseText(Phase phase) => phase switch
    {
        Phase.MoveHomeToStart => "Phase: HOME -> START",
        Phase.PickAtStart => "Phase: PICK at START",
        Phase.MoveStartToGoal => "Phase: START -> GOAL (ball attached)",
        Phase.PlaceAtGoal => "Phase: PLACE at GOAL",
        Phase.ReturnGoalToHome => "Phase: GOAL -> HOME",
        Phase.WaitAtHomeReset => "Phase: WAIT then LOOP",
        Phase.Error => "Phase: ERROR",
        _ => ""
    };
}
