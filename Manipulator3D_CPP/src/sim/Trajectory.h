#pragma once
#include <raylib.h>

namespace sim {

class LinearTrajectory {
public:
    void Reset(const Vector3& from, const Vector3& to, float durationSec);
    void Update(float dt);
    Vector3 Position() const;
    bool Finished() const { return finished_; }
    float Alpha() const { return alpha_; }

private:
    Vector3 a_{0,0,0};
    Vector3 b_{0,0,0};
    float duration_ = 1.0f;
    float t_ = 0.0f;
    float alpha_ = 0.0f;
    bool finished_ = true;
};

} // namespace sim
