#include <raylib.h>
#include <raymath.h>

#include <iostream>
#include <sstream>
#include <string>

#include "robot/RobotArm.h"
#include "sim/Trajectory.h"
#include "render/DrawUtils.h"
#include "ui/Overlay.h"

static bool TryParseVec3(const std::string& line, Vector3& out) {
    std::istringstream iss(line);
    float x, y, z;
    if (!(iss >> x >> y >> z)) return false;
    out = {x, y, z};
    return true;
}

static Vector3 PromptVec3(const char* label, Vector3 def) {
    while (true) {
        std::cout << label
                  << "  (format: x y z, z>=0; Enter=default "
                  << def.x << " " << def.y << " " << def.z << "): ";

        std::string line;
        if (!std::getline(std::cin, line)) return def;

        if (line.empty()) return def;

        Vector3 v{};
        if (!TryParseVec3(line, v)) {
            std::cout << "Invalid format. Example:  1 2 1\n";
            continue;
        }
        if (v.z < 0.0f) {
            std::cout << "Invalid input: z must be >= 0\n";
            continue;
        }
        return v;
    }
}

static Font LoadBestUIFont(int px) {
    const char* candidates[] = {
        "resources/fonts/Inter-Regular.ttf",
        "../resources/fonts/Inter-Regular.ttf",
        "C:/Windows/Fonts/segoeui.ttf",
        "C:/Windows/Fonts/arial.ttf"
    };

    for (auto path : candidates) {
        if (FileExists(path)) {
            Font f = LoadFontEx(path, px, nullptr, 0);
            if (f.texture.id != 0) {
                SetTextureFilter(f.texture, TEXTURE_FILTER_BILINEAR);
                return f;
            }
        }
    }

    Font def = GetFontDefault();
    SetTextureFilter(def.texture, TEXTURE_FILTER_BILINEAR);
    return def;
}

static void UpdateZoom(Camera3D& cam) {
    float wheel = GetMouseWheelMove();
    if (wheel == 0.0f) return;

    Vector3 v = Vector3Subtract(cam.position, cam.target);
    float dist = Vector3Length(v);
    if (dist < 0.001f) dist = 0.001f;

    float scale = 1.0f - wheel * 0.10f;
    scale = Clamp(scale, 0.70f, 1.30f);

    dist *= scale;
    dist = Clamp(dist, 1.0f, 200.0f);

    Vector3 dir = Vector3Normalize(v);
    cam.position = Vector3Add(cam.target, Vector3Scale(dir, dist));
}

enum class Phase {
    MoveHomeToStart,
    PickAtStart,
    MoveStartToGoal,
    PlaceAtGoal,
    ReturnGoalToHome,
    WaitAtHomeReset,
    Error
};

enum class BallState {
    AtStart,
    Attached,
    AtGoal
};

int main() {
    SetTraceLogLevel(LOG_ERROR);
    SetConfigFlags(FLAG_WINDOW_RESIZABLE | FLAG_MSAA_4X_HINT);

    // --- Arm parameters ---
    robot::LinkParams link1;
    link1.length_m = 3.0f;
    link1.mass_kg  = 2.0f;
    link1.RecomputeInertia();

    robot::LinkParams link2;
    link2.length_m = 2.6f;
    link2.mass_kg  = 1.6f;
    link2.RecomputeInertia();

    robot::RobotArm arm(link1, link2);

    // Defaults
    Vector3 start{1, 2, 1};
    Vector3 goal {2, 3, 2};

    // Fixed EE "home" position required by you
    const Vector3 homeEE{2.0f, 2.0f, 2.0f};

    std::cout << "Manipulator3D IK Pick&Place\n";
    std::cout << "INPUT FORMAT: x y z  (three numbers separated by spaces)\n";
    std::cout << "Example:  1 2 1\n";
    std::cout << "Axis rule: z must be >= 0 ; x and y can be negative.\n";
    std::cout << "Workspace: |p| in [" << arm.MinReach() << ", " << arm.MaxReach() << "] meters\n";
    std::cout << "Fixed EE HOME position: (2 2 2)\n\n";

    start = PromptVec3("Enter START", start);
    goal  = PromptVec3("Enter GOAL ", goal);

    // Validate endpoints + home
    auto ikHome  = arm.SolveIK(homeEE, false);
    auto ikStart = arm.SolveIK(start,  false);
    auto ikGoal  = arm.SolveIK(goal,   false);

    std::cout << "\nUsing:\n";
    std::cout << "  HOME  = (" << homeEE.x << ", " << homeEE.y << ", " << homeEE.z << ") -> "
              << (ikHome.reachable ? "reachable" : "NOT reachable") << "\n";
    if (!ikHome.reachable) std::cout << "    reason: " << ikHome.message << "\n";

    std::cout << "  START = (" << start.x << ", " << start.y << ", " << start.z << ") -> "
              << (ikStart.reachable ? "reachable" : "NOT reachable") << "\n";
    if (!ikStart.reachable) std::cout << "    reason: " << ikStart.message << "\n";

    std::cout << "  GOAL  = (" << goal.x << ", " << goal.y << ", " << goal.z << ") -> "
              << (ikGoal.reachable ? "reachable" : "NOT reachable") << "\n";
    if (!ikGoal.reachable) std::cout << "    reason: " << ikGoal.message << "\n\n";

    InitWindow(1280, 720, "Manipulator3D - 3DOF IK Pick&Place");
    SetWindowMinSize(960, 540);
    SetTargetFPS(60);

    Font uiFont = LoadBestUIFont(22);

    // Camera framing
    float reach = arm.MaxReach();
    Camera3D cam{};
    cam.target = {0.0f, 0.0f, 0.35f * reach};
    cam.up     = {0.0f, 0.0f, 1.0f};
    cam.fovy   = 52.0f;
    cam.projection = CAMERA_PERSPECTIVE;
    cam.position = { 1.10f * reach, -1.15f * reach, 0.85f * reach };

    bool paused = false;

    // Timing
    const float moveHomeToStart = 2.2f;
    const float pickDuration    = 0.45f;
    const float moveStartToGoal = 2.6f;
    const float placeDuration   = 0.35f;
    const float returnToHome    = 2.0f;
    const float resetWaitTotal  = 1.5f;  // after placing, 1.5s later the loop restarts

    float timer = 0.0f;

    // Ball size (reduced a bit)
    const float ballRadius = Clamp(0.03f * reach, 0.06f, 0.16f);

    // Ball state
    BallState ballState = BallState::AtStart;
    Vector3 ballPos = start;

    // EE control
    Vector3 targetEE = homeEE;
    robot::JointAngles qcmd{};

    // Trajectory
    sim::LinearTrajectory traj;

    Phase phase = Phase::MoveHomeToStart;

    // If anything is invalid, go error
    if (!ikHome.reachable || !ikStart.reachable || !ikGoal.reachable) {
        phase = Phase::Error;
    } else {
        // Start the arm at HOME pose
        qcmd = ikHome.q;
        targetEE = homeEE;

        // Ball starts at START
        ballState = BallState::AtStart;
        ballPos = start;

        // Begin: HOME -> START
        traj.Reset(homeEE, start, moveHomeToStart);
        phase = Phase::MoveHomeToStart;
        timer = 0.0f;
    }

    while (!WindowShouldClose()) {
        if (IsKeyPressed(KEY_F11)) ToggleFullscreen();

        int screenW = GetScreenWidth();
        int screenH = GetScreenHeight();

        UpdateZoom(cam);

        float dt = paused ? 0.0f : GetFrameTime();
        const char* runtimeError = nullptr;

        if (phase != Phase::Error) {
            switch (phase) {
                case Phase::MoveHomeToStart: {
                    // Ball stays at start, not attached yet
                    ballState = BallState::AtStart;
                    ballPos = start;

                    traj.Update(dt);
                    targetEE = traj.Position();

                    if (traj.Finished()) {
                        phase = Phase::PickAtStart;
                        timer = 0.0f;
                        targetEE = start;
                    }
                } break;

                case Phase::PickAtStart: {
                    // EE at start, ball at start
                    targetEE = start;
                    ballState = BallState::AtStart;
                    ballPos = start;

                    timer += dt;
                    if (timer >= pickDuration) {
                        // Attach and go to goal
                        ballState = BallState::Attached;
                        timer = 0.0f;
                        traj.Reset(start, goal, moveStartToGoal);
                        phase = Phase::MoveStartToGoal;
                    }
                } break;

                case Phase::MoveStartToGoal: {
                    traj.Update(dt);
                    targetEE = traj.Position();
                    ballState = BallState::Attached;

                    if (traj.Finished()) {
                        phase = Phase::PlaceAtGoal;
                        timer = 0.0f;
                        targetEE = goal;
                    }
                } break;

                case Phase::PlaceAtGoal: {
                    // EE at goal
                    targetEE = goal;

                    timer += dt;
                    if (timer >= placeDuration) {
                        // Detach ball at goal, start reset timer
                        ballState = BallState::AtGoal;
                        timer = 0.0f; // reuse as "time since place"
                        traj.Reset(goal, homeEE, returnToHome);
                        phase = Phase::ReturnGoalToHome;
                    } else {
                        // still attached during place settle
                        ballState = BallState::Attached;
                    }
                } break;

                case Phase::ReturnGoalToHome: {
                    // Ball stays at goal while arm returns
                    ballState = BallState::AtGoal;

                    traj.Update(dt);
                    targetEE = traj.Position();

                    timer += dt; // time since place
                    if (traj.Finished()) {
                        // Arm is at home; if 1.5s since place already passed -> reset now,
                        // else wait the remaining time at home.
                        phase = Phase::WaitAtHomeReset;
                    }
                } break;

                case Phase::WaitAtHomeReset: {
                    // Arm fixed at home
                    targetEE = homeEE;

                    // Continue counting time since place (timer already includes return time)
                    timer += dt;

                    if (timer >= resetWaitTotal) {
                        // Reset: ball returns to start; restart loop from HOME -> START
                        ballState = BallState::AtStart;
                        ballPos = start;

                        timer = 0.0f;
                        traj.Reset(homeEE, start, moveHomeToStart);
                        phase = Phase::MoveHomeToStart;
                    } else {
                        // Ball remains at goal until reset moment
                        ballState = BallState::AtGoal;
                    }
                } break;

                default: break;
            }

            // Solve IK for current EE target
            auto ikNow = arm.SolveIK(targetEE, false);
            if (!ikNow.reachable) {
                phase = Phase::Error;
                runtimeError = ikNow.message.c_str();
            } else {
                qcmd = ikNow.q;
            }
        }

        // FK for rendering and for the "mounted" ball offset
        auto fk = arm.ForwardKinematics(qcmd);

        // Compute tool approach direction
        Vector3 approach = Vector3Subtract(fk.ee, fk.joint2);
        float alen = Vector3Length(approach);
        if (alen > 1e-6f) approach = Vector3Scale(approach, 1.0f / alen);
        else approach = {1,0,0};

        // Ball position by state (ball always visible)
        if (ballState == BallState::AtStart) {
            ballPos = start;
        } else if (ballState == BallState::AtGoal) {
            ballPos = goal;
        } else { // Attached
            // Offset slightly forward so it looks suction-mounted
            ballPos = Vector3Add(fk.ee, Vector3Scale(approach, 0.22f));
        }

        BeginDrawing();
        ClearBackground(Color{10, 12, 16, 255});

        BeginMode3D(cam);

        //DrawGrid(24, 1.0f);
        // Thicker axes using cylinders (about 2x thickness vs line)
        const float axisLen = 3.0f;
        const float axisR   = 0.03f; // increase a bit more if you want thicker

        DrawCylinderEx({0,0,0}, {axisLen,0,0}, axisR, axisR, 12, RED);
        DrawCylinderEx({0,0,0}, {0,axisLen,0}, axisR, axisR, 12, GREEN);
        DrawCylinderEx({0,0,0}, {0,0,axisLen}, axisR, axisR, 12, BLUE);


        // Robot visuals
        render::DrawRobotBasePedestal({0,0,0});
        render::DrawRobotJointHousing(fk.base, 0.30f);
        render::DrawRobotJointHousing(fk.joint2, 0.24f);
        render::DrawRobotJointHousing(fk.ee, 0.18f);

        render::DrawTaperedLink(fk.base,   fk.joint2, 0.14f, 0.12f, Color{185,185,190,255});
        render::DrawTaperedLink(fk.joint2, fk.ee,     0.12f, 0.10f, Color{170,170,175,255});

        render::DrawSuctionTool(fk.ee, approach);

        // Ball (smaller + outline)
        DrawSphere(ballPos, ballRadius, RED);
        DrawSphereWires(ballPos, ballRadius * 1.02f, 10, 10, RAYWHITE);

        EndMode3D();

        ui::OverlayStatus st{};
        st.startReachable = ikStart.reachable;
        st.endReachable   = ikGoal.reachable;
        st.errorText      = runtimeError;

        switch (phase) {
            case Phase::MoveHomeToStart: st.phaseText = "Phase: HOME -> START"; break;
            case Phase::PickAtStart:     st.phaseText = "Phase: PICK at START"; break;
            case Phase::MoveStartToGoal: st.phaseText = "Phase: START -> GOAL (ball attached)"; break;
            case Phase::PlaceAtGoal:     st.phaseText = "Phase: PLACE at GOAL"; break;
            case Phase::ReturnGoalToHome:st.phaseText = "Phase: GOAL -> HOME"; break;
            case Phase::WaitAtHomeReset: st.phaseText = "Phase: WAIT then LOOP"; break;
            case Phase::Error:           st.phaseText = "Phase: ERROR"; break;
        }

        if (phase == Phase::Error && !runtimeError) {
            st.errorText = "ERROR: HOME/START/GOAL invalid or out of reach (z>=0 required).";
        }

        paused = ui::DrawOverlayPanel(uiFont, arm, start, goal, st, paused, screenW, screenH);

        render::DrawTextSmall(uiFont, "F11: fullscreen   Mouse Wheel: zoom", 12, screenH - 28, 18, Color{200,200,200,220});

        EndDrawing();
    }

    if (uiFont.texture.id != GetFontDefault().texture.id) UnloadFont(uiFont);
    CloseWindow();
    return 0;
}
