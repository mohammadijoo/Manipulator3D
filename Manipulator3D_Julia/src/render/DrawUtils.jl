module Render

using Raylib
using ColorTypes
using FixedPointNumbers: N0f8
using ..Robot: Vec3, v3

export draw_text_bold, draw_text_small,
       draw_robot_base_pedestal, draw_robot_joint_housing,
       draw_tapered_link, draw_suction_tool

rgba(r::Integer, g::Integer, b::Integer, a::Integer=255) =
    RGBA{N0f8}(N0f8(r/255), N0f8(g/255), N0f8(b/255), N0f8(a/255))

# Use binding-native vector types if present
const V2 = isdefined(Raylib, :RayVector2) ? getfield(Raylib, :RayVector2) : nothing
const V3 = isdefined(Raylib, :RayVector3) ? getfield(Raylib, :RayVector3) : nothing

rl_vec2(x::Real, y::Real) = (V2 === nothing) ? error("RayVector2 not found in Raylib binding.") :
                            V2(Float32(x), Float32(y))
rl_vec3(x::Real, y::Real, z::Real) = (V3 === nothing) ? error("RayVector3 not found in Raylib binding.") :
                                V3(Float32(x), Float32(y), Float32(z))

rl_v3(v::Vec3) = rl_vec3(v.x, v.y, v.z)

# -----------------------------
# Text
# -----------------------------
function draw_text_small(font, text::AbstractString, x::Int, y::Int, fontSize::Real, color)
    # integer positions reduce shimmer/jaggies in many raylib builds
    Raylib.DrawTextEx(font, String(text), rl_vec2(Int(x), Int(y)), Float32(fontSize), 1.0f0, color)
end


function draw_text_bold(font, text::AbstractString, x::Int, y::Int, fontSize::Real, color)
    draw_text_small(font, text, x, y, fontSize, color)
end


# -----------------------------
# Robot visuals
# -----------------------------
function draw_robot_base_pedestal(origin::Vec3)
    a = v3(origin.x, origin.y, -0.25f0)
    b = v3(origin.x, origin.y,  0.00f0)
    Raylib.DrawCylinderEx(rl_v3(a), rl_v3(b), 0.55f0, 0.55f0, 24, rgba(70,70,75,255))

    c = v3(origin.x, origin.y, 0.00f0)
    d = v3(origin.x, origin.y, 0.35f0)
    Raylib.DrawCylinderEx(rl_v3(c), rl_v3(d), 0.38f0, 0.34f0, 24, rgba(95,95,100,255))

    Raylib.DrawCylinderEx(rl_v3(v3(origin.x, origin.y, 0.00f0)),
                          rl_v3(v3(origin.x, origin.y, 0.06f0)),
                          0.48f0, 0.48f0, 24, rgba(110,110,115,255))
end

function draw_robot_joint_housing(center::Vec3, radius::Real)
    r = Float32(radius)
    Raylib.DrawSphere(rl_v3(center), r, rgba(120,120,125,255))
    Raylib.DrawSphereWires(rl_v3(center), r, 12, 12, rgba(200,200,200,60))

    Raylib.DrawCylinderEx(
        rl_v3(v3(center.x, center.y, center.z - 0.10f0)),
        rl_v3(v3(center.x, center.y, center.z + 0.10f0)),
        r * 0.55f0, r * 0.55f0, 18,
        rgba(85,85,90,255)
    )
end

function draw_tapered_link(a::Vec3, b::Vec3, rA::Real, rB::Real, color)
    Raylib.DrawCylinderEx(rl_v3(a), rl_v3(b), Float32(rA), Float32(rB), 20, color)
    Raylib.DrawSphere(rl_v3(a), Float32(rA) * 0.95f0, rgba(140,140,145,255))
    Raylib.DrawSphere(rl_v3(b), Float32(rB) * 0.95f0, rgba(140,140,145,255))
end

function draw_suction_tool(ee::Vec3, approachDir::Vec3)
    tip = ee + 0.28f0 * approachDir

    Raylib.DrawCylinderEx(rl_v3(ee), rl_v3(tip), 0.06f0, 0.05f0, 18, rgba(40,40,45,255))

    cupA = tip
    cupB = tip + 0.06f0 * approachDir
    Raylib.DrawCylinderEx(rl_v3(cupA), rl_v3(cupB), 0.11f0, 0.11f0, 24, rgba(25,25,28,255))

    Raylib.DrawSphere(rl_v3(tip), 0.035f0, rgba(80,80,85,255))
end

end # module Render
