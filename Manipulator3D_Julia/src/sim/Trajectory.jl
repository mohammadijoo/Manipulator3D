module Sim

export LinearTrajectory, reset!, update!, position, finished, alpha

using ..Robot: Vec3, v3

mutable struct LinearTrajectory
    a::Vec3
    b::Vec3
    duration::Float32
    t::Float32
    α::Float32
    done::Bool
end

function LinearTrajectory()
    return LinearTrajectory(v3(0,0,0), v3(0,0,0), 1.0f0, 0.0f0, 0.0f0, true)
end

function reset!(tr::LinearTrajectory, from::Vec3, to::Vec3, durationSec::Real)
    tr.a = from
    tr.b = to
    d = Float32(durationSec)
    tr.duration = (d > 1f-6) ? d : 1f-6
    tr.t = 0.0f0
    tr.α = 0.0f0
    tr.done = false
    return nothing
end

function update!(tr::LinearTrajectory, dt::Float32)
    tr.done && return nothing
    tr.t += dt
    tr.α = clamp(tr.t / tr.duration, 0.0f0, 1.0f0)
    if tr.α >= 1.0f0
        tr.done = true
    end
    return nothing
end

function position(tr::LinearTrajectory)::Vec3
    a = tr.a
    b = tr.b
    α = tr.α
    return v3(
        a.x + (b.x - a.x) * α,
        a.y + (b.y - a.y) * α,
        a.z + (b.z - a.z) * α
    )
end

finished(tr::LinearTrajectory) = tr.done
alpha(tr::LinearTrajectory) = tr.α

end # module Sim
