module UI

using Raylib
using Printf
using ColorTypes
using FixedPointNumbers: N0f8
using ..Robot: Vec3, v3, norm3, RobotArm, solve_ik, max_reach, min_reach
using ..Render: draw_text_bold, draw_text_small

export OverlayStatus, OverlayModel, draw_overlay!

rgba(r::Integer, g::Integer, b::Integer, a::Integer=255) =
    RGBA{N0f8}(N0f8(r/255), N0f8(g/255), N0f8(b/255), N0f8(a/255))

# -----------------------
# Input constants (Raylib.jl v0.2.0 expects Integer codes)
# -----------------------
const MB_LEFT       = 0          # mouse left
const KEY_BACKSPACE = 259
const KEY_ENTER     = 257
const KEY_KP_ENTER  = 335

struct OverlayStatus
    start_reachable::Bool
    goal_reachable::Bool
    error_text::Union{Nothing,String}
    phase_text::String
end

mutable struct TextField
    text::String
    x::Float32
    y::Float32
    w::Float32
    h::Float32
    active::Bool
    maxlen::Int
end

mutable struct OverlayModel
    start_fx::TextField
    start_fy::TextField
    start_fz::TextField
    goal_fx::TextField
    goal_fy::TextField
    goal_fz::TextField
    message::String
end

# -----------------------
# Internal helpers
# -----------------------

vx(v) = Base.hasproperty(v, :x) ? getproperty(v, :x) : v[1]
vy(v) = Base.hasproperty(v, :y) ? getproperty(v, :y) : v[2]

struct Rect
    x::Float32
    y::Float32
    w::Float32
    h::Float32
end

function point_in_rect(p, r::Rect)
    px = Float32(vx(p))
    py = Float32(vy(p))
    return (px >= r.x && px <= r.x + r.w &&
            py >= r.y && py <= r.y + r.h)
end

function draw_rect(r::Rect, c)
    Raylib.DrawRectangle(Int(round(r.x)), Int(round(r.y)),
                         Int(round(r.w)), Int(round(r.h)), c)
end

function draw_rect_lines(r::Rect, c)
    Raylib.DrawRectangleLines(Int(round(r.x)), Int(round(r.y)),
                              Int(round(r.w)), Int(round(r.h)), c)
end

function draw_rect_border_thick(r::Rect, thick::Int, c)
    t = max(thick, 1)
    x = Int(round(r.x)); y = Int(round(r.y))
    w = Int(round(r.w)); h = Int(round(r.h))

    Raylib.DrawRectangle(x, y, w, t, c)
    Raylib.DrawRectangle(x, y + h - t, w, t, c)
    Raylib.DrawRectangle(x, y, t, h, c)
    Raylib.DrawRectangle(x + w - t, y, t, h, c)
end

# FIX 1: String(::Char) is invalid → use string(c)
function sanitize_char(codepoint::Int32)
    c = Char(codepoint)
    if c in ('0':'9') || c in ('.', '-', '+', 'e', 'E')
        return string(c)
    end
    return ""
end

function update_text_field!(tf::TextField; enabled::Bool)
    mp = Raylib.GetMousePosition()
    r = Rect(tf.x, tf.y, tf.w, tf.h)

    if enabled && Raylib.IsMouseButtonPressed(MB_LEFT) && point_in_rect(mp, r)
        tf.active = true
    elseif enabled && Raylib.IsMouseButtonPressed(MB_LEFT) && !point_in_rect(mp, r)
        tf.active = false
    end

    if enabled && tf.active
        key = Raylib.GetCharPressed()
        while key > 0
            if length(tf.text) < tf.maxlen
                s = sanitize_char(Int32(key))
                if !isempty(s)
                    tf.text *= s
                end
            end
            key = Raylib.GetCharPressed()
        end

        if Raylib.IsKeyPressed(KEY_BACKSPACE) && !isempty(tf.text)
            tf.text = tf.text[1:prevind(tf.text, lastindex(tf.text))]
        end

        if Raylib.IsKeyPressed(KEY_ENTER) || Raylib.IsKeyPressed(KEY_KP_ENTER)
            tf.active = false
        end
    end

    return nothing
end

function draw_text_field!(font, tf::TextField; enabled::Bool)
    r = Rect(tf.x, tf.y, tf.w, tf.h)

    bg     = enabled ? rgba(30,30,30,220) : rgba(22,22,22,220)
    border = tf.active ? rgba(230,230,230,255) : rgba(160,160,160,255)

    draw_rect(r, bg)
    draw_rect_border_thick(r, 2, border)

    pad = 8
    # FIX 2: ensure we always draw using the provided font
    draw_text_small(font, tf.text, Int(tf.x) + pad, Int(tf.y) + 6, 18, Raylib.RAYWHITE)

    return nothing
end

function parse_vec3_from_fields(fx::TextField, fy::TextField, fz::TextField)
    x = tryparse(Float32, strip(fx.text))
    y = tryparse(Float32, strip(fy.text))
    z = tryparse(Float32, strip(fz.text))
    (x === nothing || y === nothing || z === nothing) && return nothing
    return v3(x, y, z)
end

# Layout helper (prevents overlap)
function layout_fields!(model::OverlayModel, x::Int, y::Int; fw=90.0f0, fh=30.0f0, gap=8.0f0)
    x0 = Float32(x)
    y0 = Float32(y)
    model.start_fx.x = x0
    model.start_fy.x = x0 + (fw + gap)*1
    model.start_fz.x = x0 + (fw + gap)*2

    model.start_fx.y = y0
    model.start_fy.y = y0
    model.start_fz.y = y0

    model.start_fx.w = fw; model.start_fy.w = fw; model.start_fz.w = fw
    model.start_fx.h = fh; model.start_fy.h = fh; model.start_fz.h = fh

    return nothing
end

function layout_fields_goal!(model::OverlayModel, x::Int, y::Int; fw=90.0f0, fh=30.0f0, gap=8.0f0)
    x0 = Float32(x)
    y0 = Float32(y)
    model.goal_fx.x = x0
    model.goal_fy.x = x0 + (fw + gap)*1
    model.goal_fz.x = x0 + (fw + gap)*2

    model.goal_fx.y = y0
    model.goal_fy.y = y0
    model.goal_fz.y = y0

    model.goal_fx.w = fw; model.goal_fy.w = fw; model.goal_fz.w = fw
    model.goal_fx.h = fh; model.goal_fy.h = fh; model.goal_fz.h = fh

    return nothing
end

# -----------------------
# Public constructors
# -----------------------

function OverlayModel(start::Vec3, goal::Vec3)
    tf(txt) = TextField(txt, 0.0f0, 0.0f0, 90.0f0, 30.0f0, false, 12)

    start_fx = tf(@sprintf("%.2f", start.x))
    start_fy = tf(@sprintf("%.2f", start.y))
    start_fz = tf(@sprintf("%.2f", start.z))

    goal_fx  = tf(@sprintf("%.2f", goal.x))
    goal_fy  = tf(@sprintf("%.2f", goal.y))
    goal_fz  = tf(@sprintf("%.2f", goal.z))

    return OverlayModel(start_fx, start_fy, start_fz, goal_fx, goal_fy, goal_fz, "")
end

"""
Returns:
  (paused, new_start, new_goal, restart_requested)
"""
function draw_overlay!(
    model::OverlayModel,
    font,
    arm::RobotArm,
    start::Vec3,
    goal::Vec3,
    status::OverlayStatus,
    paused::Bool,
    screenW::Int,
    screenH::Int
)
    pad = 12
    x0 = 14
    y0 = 14
    w  = 380
    h  = 520   # a bit taller to avoid crowding

    panel = Rect(Float32(x0), Float32(y0), Float32(w), Float32(h))
    draw_rect(panel, rgba(18,18,18,230))
    draw_rect_lines(panel, rgba(200,200,200,255))

    x = x0 + pad
    y = y0 + pad

    draw_text_bold(font, "3-DOF 2-Link Arm", x, y, 24, Raylib.RAYWHITE)
    y += 34

    if !isempty(status.phase_text)
        draw_text_small(font, status.phase_text, x, y, 18, Raylib.SKYBLUE)
        y += 24
    end

    draw_text_small(font, "Edit START/GOAL while PAUSED, then PLAY.", x, y, 16, rgba(200,200,200,220))
    y += 24

    # Play/Pause button
    btn = Rect(Float32(x), Float32(y), Float32(w - 2*pad), 34.0f0)
    btn_bg = paused ? rgba(60,120,60,220) : rgba(120,60,60,220)
    draw_rect(btn, btn_bg)
    draw_rect_border_thick(btn, 2, rgba(230,230,230,255))

    label = paused ? "PLAY" : "PAUSE"
    draw_text_bold(font, label, Int(btn.x) + 10, Int(btn.y) + 6, 22, Raylib.RAYWHITE)

    restart_requested = false
    clicked = Raylib.IsMouseButtonPressed(MB_LEFT) && point_in_rect(Raylib.GetMousePosition(), btn)

    if clicked
        if paused
            p_start = parse_vec3_from_fields(model.start_fx, model.start_fy, model.start_fz)
            p_goal  = parse_vec3_from_fields(model.goal_fx,  model.goal_fy,  model.goal_fz)

            if p_start === nothing || p_goal === nothing
                model.message = "Invalid input: use numeric values like 1.0 or -2.5"
                paused = true
            elseif p_start.z < 0.0f0 || p_goal.z < 0.0f0
                model.message = "Invalid input: z must be >= 0"
                paused = true
            else
                ikS = solve_ik(arm, p_start)
                ikG = solve_ik(arm, p_goal)

                if !ikS.reachable
                    model.message = "START not reachable: " * ikS.message
                    paused = true
                elseif !ikG.reachable
                    model.message = "GOAL not reachable: " * ikG.message
                    paused = true
                else
                    model.message = ""
                    paused = false
                    start = p_start
                    goal  = p_goal
                    restart_requested = true
                end
            end
        else
            paused = true
        end
    end

    y += 48

    # Link parameters (compact spacing)
    L1 = arm.link1
    L2 = arm.link2

    draw_text_small(font, @sprintf("Link1: L=%.3fm  m=%.3fkg", L1.length_m, L1.mass_kg), x, y, 18, Raylib.RAYWHITE); y += 20
    draw_text_small(font, @sprintf("Link1 inertia (joint): %.5f", L1.inertia_joint), x, y, 18, Raylib.RAYWHITE); y += 22
    draw_text_small(font, @sprintf("Link2: L=%.3fm  m=%.3fkg", L2.length_m, L2.mass_kg), x, y, 18, Raylib.RAYWHITE); y += 20
    draw_text_small(font, @sprintf("Link2 inertia (joint): %.5f", L2.inertia_joint), x, y, 18, Raylib.RAYWHITE); y += 24

    rmin = min_reach(arm)
    rmax = max_reach(arm)
    draw_text_small(font, @sprintf("Workspace |p|: [%.2f, %.2f]m", rmin, rmax), x, y, 18, rgba(140,200,255,255))
    y += 28

    # START fields
    draw_text_small(font, "START (x, y, z):", x, y, 18, Raylib.RAYWHITE)
    y += 22

    layout_fields!(model, x, y)
    enabled = paused

    update_text_field!(model.start_fx; enabled=enabled)
    update_text_field!(model.start_fy; enabled=enabled)
    update_text_field!(model.start_fz; enabled=enabled)

    draw_text_field!(font, model.start_fx; enabled=enabled)
    draw_text_field!(font, model.start_fy; enabled=enabled)
    draw_text_field!(font, model.start_fz; enabled=enabled)

    y += Int(round(model.start_fx.h)) + 20

    # GOAL fields
    draw_text_small(font, "GOAL  (x, y, z):", x, y, 18, Raylib.RAYWHITE)
    y += 22

    layout_fields_goal!(model, x, y)
    update_text_field!(model.goal_fx; enabled=enabled)
    update_text_field!(model.goal_fy; enabled=enabled)
    update_text_field!(model.goal_fz; enabled=enabled)

    draw_text_field!(font, model.goal_fx; enabled=enabled)
    draw_text_field!(font, model.goal_fy; enabled=enabled)
    draw_text_field!(font, model.goal_fz; enabled=enabled)

    y += Int(round(model.goal_fx.h)) + 18

    # Status summary
    ds = norm3(start)
    dg = norm3(goal)

    s_col = status.start_reachable ? Raylib.GREEN : Raylib.ORANGE
    g_col = status.goal_reachable  ? Raylib.GREEN : Raylib.ORANGE

    draw_text_small(font, @sprintf("Start |p|=%.3f", ds), x, y, 18, s_col); y += 20
    draw_text_small(font, @sprintf("Goal  |p|=%.3f", dg), x, y, 18, g_col); y += 22

    if !status.start_reachable || !status.goal_reachable
        draw_text_bold(font, "OUT OF REACH!", x, y, 22, Raylib.RED)
        y += 24
        draw_text_small(font, "Choose points inside workspace.", x, y, 18, Raylib.RED)
        y += 22
    end

    if status.error_text !== nothing
        draw_text_bold(font, status.error_text, x, y, 20, Raylib.RED)
        y += 22
    end

    if !isempty(model.message)
        draw_text_small(font, model.message, x, y, 18, Raylib.RED)
    end

    return paused, start, goal, restart_requested
end

end # module UI
