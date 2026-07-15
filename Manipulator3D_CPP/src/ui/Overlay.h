#pragma once
#include <raylib.h>
#include "robot/RobotArm.h"

namespace ui {

struct OverlayStatus {
    bool startReachable = false;
    bool endReachable = false;
    const char* errorText = nullptr;
    const char* phaseText = "";
};

// Returns updated paused state (toggle via button click)
bool DrawOverlayPanel(
    Font font,
    const robot::RobotArm& arm,
    Vector3 start,
    Vector3 goal,
    const OverlayStatus& status,
    bool paused,
    int screenW,
    int screenH
);

} // namespace ui
