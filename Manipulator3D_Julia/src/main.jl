# Manipulator3D — 3-DOF Two-Link Robot Arm (Analytic IK + Pick&Place)
#
# Run:
#   julia --project=. src/main.jl

using Raylib
using Printf
using ColorTypes
using FixedPointNumbers: N0f8

include(joinpath(@__DIR__, "robot", "RobotArm.jl"))
include(joinpath(@__DIR__, "sim", "Trajectory.jl"))
include(joinpath(@__DIR__, "render", "DrawUtils.jl"))
include(joinpath(@__DIR__, "ui", "Overlay.jl"))

using .Robot
using .Render
using .UI
using .Sim: LinearTrajectory, reset!, update!, finished

# -----------------------------
# Color helper
# -----------------------------
rgba(r::Integer, g::Integer, b::Integer, a::Integer=255) =
    RGBA{N0f8}(N0f8(r/255), N0f8(g/255), N0f8(b/255), N0f8(a/255))

# -----------------------------
# Raylib binding types (from your environment)
# -----------------------------
const CAM3D_T = Raylib.RayCamera3D
const VEC3_T  = Raylib.RayVector3

rl_vec3(x::Real, y::Real, z::Real) = VEC3_T(Float32(x), Float32(y), Float32(z))
rl_v3(v::Vec3) = rl_vec3(v.x, v.y, v.z)

# -----------------------------
# Camera model
# -----------------------------
mutable struct CameraModel
    position::Vec3
    target::Vec3
    up::Vec3
    fovy::Float32
    projection::Int32
end

function _zero_like(ft::Type)
    if ft <: AbstractFloat
        return ft(0)
    elseif ft <: Integer
        return ft(0)
    else
        try
            return zero(ft)
        catch
            return 0
        end
    end
end

# Build RayCamera3D for both:
# A) nested Vector3 layout: (Vector3, Vector3, Vector3, Float32, Int32)
# B) flattened layout: (Float32 x10, Int32) -> pos(3), target(3), up(3), fovy, projection
function camera_to_raylib(cam::CameraModel)
    fts = fieldtypes(CAM3D_T)
    nf  = length(fts)

    is_vec3like(T) = (T isa DataType) && (nfields(T) == 3) && all(t -> t <: AbstractFloat, fieldtypes(T))

    # --- Layout A: nested vectors
    if nf >= 5 && is_vec3like(fts[1]) && is_vec3like(fts[2]) && is_vec3like(fts[3]) &&
       (fts[4] <: AbstractFloat) && (fts[5] <: Integer)

        V3 = fts[1]
        makev3(x,y,z) = V3(Float32(x), Float32(y), Float32(z))

        pos = makev3(cam.position.x, cam.position.y, cam.position.z)
        tgt = makev3(cam.target.x,   cam.target.y,   cam.target.z)
        upv = makev3(cam.up.x,       cam.up.y,       cam.up.z)

        args = Vector{Any}(undef, nf)
        args[1] = pos
        args[2] = tgt
        args[3] = upv
        args[4] = Float32(cam.fovy)
        args[5] = Int32(cam.projection)
        for i in 6:nf
            args[i] = _zero_like(fts[i])
        end
        return CAM3D_T(args...)
    end

    # --- Layout B: flattened scalars (your binding)
    if nf >= 11 &&
       all(t -> t <: AbstractFloat, fts[1:10]) &&
       (fts[11] <: Integer)

        args = Vector{Any}(undef, nf)

        args[1]  = Float32(cam.position.x)
        args[2]  = Float32(cam.position.y)
        args[3]  = Float32(cam.position.z)

        args[4]  = Float32(cam.target.x)
        args[5]  = Float32(cam.target.y)
        args[6]  = Float32(cam.target.z)

        args[7]  = Float32(cam.up.x)
        args[8]  = Float32(cam.up.y)
        args[9]  = Float32(cam.up.z)

        args[10] = Float32(cam.fovy)
        args[11] = Int32(cam.projection)

        for i in 12:nf
            args[i] = _zero_like(fts[i])
        end

        return CAM3D_T(args...)
    end

    error("Unsupported RayCamera3D layout: nfields=$(nf), fieldtypes=$(fts)")
end

# -----------------------------
# Math helpers
# -----------------------------
function vec3_sub(a::Vec3, b::Vec3)::Vec3
    return v3(a.x - b.x, a.y - b.y, a.z - b.z)
end

function vec3_len(v::Vec3)::Float32
    return Float32(sqrt(v.x*v.x + v.y*v.y + v.z*v.z))
end

function vec3_norm(v::Vec3)::Vec3
    L = vec3_len(v)
    if L < 1f-6
        return v3(1,0,0)
    end
    return (1.0f0 / L) * v
end

function update_zoom!(cam::CameraModel)
    wheel = Raylib.GetMouseWheelMove()
    wheel == 0.0f0 && return

    v = vec3_sub(cam.position, cam.target)
    dist = vec3_len(v)
    dist < 1f-3 && (dist = 1f-3)

    scale = 1.0f0 - Float32(wheel) * 0.10f0
    scale = clamp(scale, 0.70f0, 1.30f0)

    dist *= scale
    dist = clamp(dist, 1.0f0, 200.0f0)

    dir = vec3_norm(v)
    cam.position = cam.target + dist * dir
    return
end

function load_best_ui_font(px::Int)
    to_abs_slash(p::AbstractString) = replace(abspath(p), '\\' => '/')

    candidates = String[
        to_abs_slash(joinpath(pwd(), "resources", "fonts", "Inter-Regular.ttf")),
        to_abs_slash(joinpath(@__DIR__, "..", "resources", "fonts", "Inter-Regular.ttf")),
        to_abs_slash("C:/Windows/Fonts/segoeui.ttf"),
        to_abs_slash("C:/Windows/Fonts/arial.ttf"),
    ]

    # Bake ASCII printable glyphs into the font atlas
    charset = Int32.(32:126)          # 95 codepoints
    charset_ref = Ref(charset, 1)     # ref to first element
    glyph_count = length(charset)

    for p in candidates
        if isfile(p)
            println("[UI FONT] found: ", p)

            f = Raylib.LoadFontEx(p, px, charset_ref, glyph_count)

            ok = false
            glyphs = -1
            try
                glyphs = getproperty(f, :glyphCount)
                ok = (getproperty(f, :baseSize) > 0) && (glyphs > 0)
            catch err
                # If fields aren't exposed, try texture id
                try
                    ok = (getproperty(getproperty(f, :texture), :id) != 0)
                catch err2
                    ok = true
                end
            end

            if ok
                try
                    Raylib.SetTextureFilter(f.texture, Raylib.TEXTURE_FILTER_BILINEAR)
                catch err3
                end
                println("[UI FONT] loaded OK: ", p, "  glyphs=", glyphs)
                return f
            else
                println("[UI FONT] load failed (invalid font object): ", p)
            end
        end
    end

    println("[UI FONT] fallback: raylib default font")
    f = Raylib.GetFontDefault()
    try
        Raylib.SetTextureFilter(f.texture, Raylib.TEXTURE_FILTER_BILINEAR)
    catch err4
    end
    return f
end


# -----------------------------
# State machine
# -----------------------------
@enum Phase begin
    MoveHomeToStart
    PickAtStart
    MoveStartToGoal
    PlaceAtGoal
    ReturnGoalToHome
    WaitAtHomeReset
    ErrorPhase
end

@enum BallState begin
    AtStart
    Attached
    AtGoal
end

# -----------------------------
# Main
# -----------------------------
function main()
    try
        Raylib.SetTraceLogLevel(Raylib.LOG_ERROR)
    catch
    end

    try
        Raylib.SetConfigFlags(Raylib.FLAG_WINDOW_RESIZABLE | Raylib.FLAG_MSAA_4X_HINT)
    catch
    end

    link1 = LinkParams(length_m=3.0, mass_kg=2.0)
    link2 = LinkParams(length_m=2.6, mass_kg=1.6)
    arm = RobotArm(link1, link2)

    start = v3(1, 2, 1)
    goal  = v3(2, 3, 2)
    homeEE = v3(2, 2, 2)

    Raylib.InitWindow(1280, 720, "Manipulator3D - 3DOF IK Pick&Place (Julia)")
    try
        Raylib.SetWindowMinSize(960, 540)
    catch
    end
    Raylib.SetTargetFPS(60)

    ui_font = load_best_ui_font(28)

    reach = max_reach(arm)

    proj = Int32(0)
    try
        proj = Int32(Raylib.CAMERA_PERSPECTIVE)
    catch
        proj = Int32(0)
    end

    cam = CameraModel(
        v3( 1.10f0 * reach, -1.15f0 * reach, 0.85f0 * reach ),
        v3( 0.0, 0.0, 0.35f0 * reach ),
        v3( 0.0, 0.0, 1.0 ),
        52.0f0,
        proj
    )

    paused = true

    moveHomeToStart = 2.2f0
    pickDuration    = 0.45f0
    moveStartToGoal = 2.6f0
    placeDuration   = 0.35f0
    returnToHome    = 2.0f0
    resetWaitTotal  = 1.5f0

    timer = 0.0f0

    ballRadius = clamp(0.03f0 * reach, 0.06f0, 0.16f0)

    ballState = AtStart
    ballPos   = start

    targetEE  = homeEE
    qcmd      = JointAngles()

    traj = LinearTrajectory()

    phase = MoveHomeToStart
    runtimeError = nothing

    ikHome = solve_ik(arm, homeEE)
    if !ikHome.reachable
        phase = ErrorPhase
        runtimeError = "HOME not reachable: " * ikHome.message
    else
        qcmd = ikHome.q
        targetEE = homeEE
        reset!(traj, homeEE, start, moveHomeToStart)
        phase = MoveHomeToStart
        timer = 0.0f0
        ballState = AtStart
        ballPos = start
    end

    overlay = OverlayModel(start, goal)

    while !Raylib.WindowShouldClose()
        try
            if Raylib.IsKeyPressed(Raylib.KEY_F11)
                Raylib.ToggleFullscreen()
            end
        catch
        end

        # ✅ FIX: convert Int32 -> Int so it matches draw_overlay! signature
        screenW = Int(Raylib.GetScreenWidth())
        screenH = Int(Raylib.GetScreenHeight())

        update_zoom!(cam)

        dt = paused ? 0.0f0 : Raylib.GetFrameTime()
        runtimeError = nothing

        if phase != ErrorPhase
            if phase == MoveHomeToStart
                ballState = AtStart
                ballPos = start

                update!(traj, dt)
                targetEE = Sim.position(traj)

                if finished(traj)
                    phase = PickAtStart
                    timer = 0.0f0
                    targetEE = start
                end

            elseif phase == PickAtStart
                targetEE = start
                ballState = AtStart
                ballPos = start

                timer += dt
                if timer >= pickDuration
                    ballState = Attached
                    timer = 0.0f0
                    reset!(traj, start, goal, moveStartToGoal)
                    phase = MoveStartToGoal
                end

            elseif phase == MoveStartToGoal
                update!(traj, dt)
                targetEE = Sim.position(traj)
                ballState = Attached

                if finished(traj)
                    phase = PlaceAtGoal
                    timer = 0.0f0
                    targetEE = goal
                end

            elseif phase == PlaceAtGoal
                targetEE = goal
                timer += dt
                if timer >= placeDuration
                    ballState = AtGoal
                    timer = 0.0f0
                    reset!(traj, goal, homeEE, returnToHome)
                    phase = ReturnGoalToHome
                else
                    ballState = Attached
                end

            elseif phase == ReturnGoalToHome
                ballState = AtGoal
                update!(traj, dt)
                targetEE = Sim.position(traj)
                timer += dt
                if finished(traj)
                    phase = WaitAtHomeReset
                end

            elseif phase == WaitAtHomeReset
                targetEE = homeEE
                timer += dt
                if timer >= resetWaitTotal
                    ballState = AtStart
                    ballPos = start
                    timer = 0.0f0
                    reset!(traj, homeEE, start, moveHomeToStart)
                    phase = MoveHomeToStart
                else
                    ballState = AtGoal
                end
            end

            ikNow = solve_ik(arm, targetEE)
            if !ikNow.reachable
                phase = ErrorPhase
                runtimeError = ikNow.message
            else
                qcmd = ikNow.q
            end
        end

        fk = forward_kinematics(arm, qcmd)
        approach = vec3_norm(vec3_sub(fk.ee, fk.joint2))

        if ballState == AtStart
            ballPos = start
        elseif ballState == AtGoal
            ballPos = goal
        else
            ballPos = fk.ee + 0.22f0 * approach
        end

        ikStart = solve_ik(arm, start)
        ikGoal  = solve_ik(arm, goal)

        phase_text = phase == MoveHomeToStart   ? "Phase: HOME -> START" :
                     phase == PickAtStart      ? "Phase: PICK at START" :
                     phase == MoveStartToGoal  ? "Phase: START -> GOAL (ball attached)" :
                     phase == PlaceAtGoal      ? "Phase: PLACE at GOAL" :
                     phase == ReturnGoalToHome ? "Phase: GOAL -> HOME" :
                     phase == WaitAtHomeReset  ? "Phase: WAIT then LOOP" :
                     phase == ErrorPhase       ? "Phase: ERROR" : ""

        err_text = (phase == ErrorPhase) ? (runtimeError === nothing ? "ERROR: invalid or out of reach (z>=0 required)." : runtimeError) : nothing
        st = OverlayStatus(ikStart.reachable, ikGoal.reachable, err_text, phase_text)

        Raylib.BeginDrawing()
        Raylib.ClearBackground(rgba(10,12,16,255))

        Raylib.BeginMode3D(camera_to_raylib(cam))

        axisLen = 3.0f0
        axisR   = 0.03f0
        Raylib.DrawCylinderEx(rl_vec3(0,0,0), rl_vec3(axisLen,0,0), axisR, axisR, 12, Raylib.RED)
        Raylib.DrawCylinderEx(rl_vec3(0,0,0), rl_vec3(0,axisLen,0), axisR, axisR, 12, Raylib.GREEN)
        Raylib.DrawCylinderEx(rl_vec3(0,0,0), rl_vec3(0,0,axisLen), axisR, axisR, 12, Raylib.BLUE)

        draw_robot_base_pedestal(v3(0,0,0))
        draw_robot_joint_housing(fk.base, 0.30f0)
        draw_robot_joint_housing(fk.joint2, 0.24f0)
        draw_robot_joint_housing(fk.ee, 0.18f0)

        draw_tapered_link(fk.base, fk.joint2, 0.14f0, 0.12f0, rgba(185,185,190,255))
        draw_tapered_link(fk.joint2, fk.ee,   0.12f0, 0.10f0, rgba(170,170,175,255))

        draw_suction_tool(fk.ee, approach)

        Raylib.DrawSphere(rl_v3(ballPos), ballRadius, Raylib.RED)
        Raylib.DrawSphereWires(rl_v3(ballPos), ballRadius * 1.02f0, 10, 10, Raylib.RAYWHITE)

        Raylib.EndMode3D()

        paused, start, goal, restart = draw_overlay!(overlay, ui_font, arm, start, goal, st, paused, screenW, screenH)

        if restart
            ikHome2  = solve_ik(arm, homeEE)
            ikStart2 = solve_ik(arm, start)
            ikGoal2  = solve_ik(arm, goal)

            if !ikHome2.reachable
                phase = ErrorPhase
                runtimeError = "HOME not reachable: " * ikHome2.message
            elseif !ikStart2.reachable
                phase = ErrorPhase
                runtimeError = "START not reachable: " * ikStart2.message
            elseif !ikGoal2.reachable
                phase = ErrorPhase
                runtimeError = "GOAL not reachable: " * ikGoal2.message
            else
                qcmd = ikHome2.q
                targetEE = homeEE
                ballState = AtStart
                ballPos = start
                reset!(traj, homeEE, start, moveHomeToStart)
                phase = MoveHomeToStart
                timer = 0.0f0
                runtimeError = nothing
            end
        end

        draw_text_small(ui_font, "F11: fullscreen   Mouse Wheel: zoom", 12, screenH - 28, 18, rgba(200,200,200,220))
        Raylib.EndDrawing()
    end

    try
        if ui_font.texture.id != Raylib.GetFontDefault().texture.id
            Raylib.UnloadFont(ui_font)
        end
    catch
    end

    Raylib.CloseWindow()
    return nothing
end

main()
