#include "sim/Trajectory.h"
#include <algorithm>

namespace sim {

void LinearTrajectory::Reset(const Vector3& from, const Vector3& to, float durationSec) {
    a_ = from;
    b_ = to;
    duration_ = (durationSec > 1e-6f) ? durationSec : 1e-6f;
    t_ = 0.0f;
    alpha_ = 0.0f;
    finished_ = false;
}

void LinearTrajectory::Update(float dt) {
    if (finished_) return;
    t_ += dt;
    alpha_ = std::clamp(t_ / duration_, 0.0f, 1.0f);
    if (alpha_ >= 1.0f) finished_ = true;
}

Vector3 LinearTrajectory::Position() const {
    return {
        a_.x + (b_.x - a_.x) * alpha_,
        a_.y + (b_.y - a_.y) * alpha_,
        a_.z + (b_.z - a_.z) * alpha_
    };
}

} // namespace sim
