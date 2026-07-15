#include "ui/Overlay.h"
#include "render/DrawUtils.h"
#include <cstdio>
#include <cmath>

namespace ui {

static float norm3(Vector3 v) {
    return std::sqrt(v.x*v.x + v.y*v.y + v.z*v.z);
}

static bool PointInRect(Vector2 p, Rectangle r) {
    return (p.x >= r.x && p.x <= r.x + r.width &&
            p.y >= r.y && p.y <= r.y + r.height);
}

bool DrawOverlayPanel(
    Font font,
    const robot::RobotArm& arm,
    Vector3 start,
    Vector3 goal,
    const OverlayStatus& status,
    bool paused,
    int /*screenW*/,
    int /*screenH*/
) {
    // Half width, taller height
    const int pad = 12;
    const int x0 = 14;
    const int y0 = 14;
    const int w  = 320;
    const int h  = 360;

    DrawRectangle(x0, y0, w, h, Color{18, 18, 18, 230});
    DrawRectangleLines(x0, y0, w, h, Color{200, 200, 200, 255});

    int y = y0 + pad;

    render::DrawTextBold(font, "3-DOF 2-Link Arm", x0 + pad, y, 24, RAYWHITE);
    y += 30;

    if (status.phaseText && status.phaseText[0] != '\0') {
        render::DrawTextSmall(font, status.phaseText, x0 + pad, y, 18, SKYBLUE);
        y += 24;
    }

    // Pause/Play button
    Rectangle btn{ (float)(x0 + pad), (float)y, (float)(w - 2*pad), 34.0f };
    Color btnBg = paused ? Color{60, 120, 60, 220} : Color{120, 60, 60, 220};
    DrawRectangleRounded(btn, 0.18f, 8, btnBg);
    DrawRectangleRoundedLines(btn, 0.18f, 8, 2.0f, Color{230,230,230,255});

    const char* label = paused ? "PLAY" : "PAUSE";
    render::DrawTextBold(font, label, (int)btn.x + 10, (int)btn.y + 6, 22, RAYWHITE);

    Vector2 mp = GetMousePosition();
    if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT) && PointInRect(mp, btn)) {
        paused = !paused;
    }

    y += 46;

    const auto& L1 = arm.Link1();
    const auto& L2 = arm.Link2();

    char buf[256];

    // Split lines (so narrow panel is readable)
    std::snprintf(buf, sizeof(buf), "Link1 length: %.3fm", L1.length_m);
    render::DrawTextSmall(font, buf, x0 + pad, y, 18, RAYWHITE); y += 20;
    std::snprintf(buf, sizeof(buf), "Link1 mass  : %.3fkg", L1.mass_kg);
    render::DrawTextSmall(font, buf, x0 + pad, y, 18, RAYWHITE); y += 20;
    std::snprintf(buf, sizeof(buf), "Link1 inertia (joint): %.5f", L1.inertia_joint);
    render::DrawTextSmall(font, buf, x0 + pad, y, 18, RAYWHITE); y += 24;

    std::snprintf(buf, sizeof(buf), "Link2 length: %.3fm", L2.length_m);
    render::DrawTextSmall(font, buf, x0 + pad, y, 18, RAYWHITE); y += 20;
    std::snprintf(buf, sizeof(buf), "Link2 mass  : %.3fkg", L2.mass_kg);
    render::DrawTextSmall(font, buf, x0 + pad, y, 18, RAYWHITE); y += 20;
    std::snprintf(buf, sizeof(buf), "Link2 inertia (joint): %.5f", L2.inertia_joint);
    render::DrawTextSmall(font, buf, x0 + pad, y, 18, RAYWHITE); y += 26;

    const float rmin = arm.MinReach();
    const float rmax = arm.MaxReach();
    std::snprintf(buf, sizeof(buf), "Workspace |p|: [%.2f, %.2f]m", rmin, rmax);
    render::DrawTextSmall(font, buf, x0 + pad, y, 18, Color{140, 200, 255, 255}); y += 26;

    const float ds = norm3(start);
    const float dg = norm3(goal);

    std::snprintf(buf, sizeof(buf), "Start |p|=%.3f", ds);
    render::DrawTextSmall(font, buf, x0 + pad, y, 18, status.startReachable ? GREEN : ORANGE); y += 20;
    std::snprintf(buf, sizeof(buf), "Start: (%.2f, %.2f, %.2f)", start.x, start.y, start.z);
    render::DrawTextSmall(font, buf, x0 + pad, y, 18, status.startReachable ? GREEN : ORANGE); y += 24;

    std::snprintf(buf, sizeof(buf), "Goal  |p|=%.3f", dg);
    render::DrawTextSmall(font, buf, x0 + pad, y, 18, status.endReachable ? GREEN : ORANGE); y += 20;
    std::snprintf(buf, sizeof(buf), "Goal : (%.2f, %.2f, %.2f)", goal.x, goal.y, goal.z);
    render::DrawTextSmall(font, buf, x0 + pad, y, 18, status.endReachable ? GREEN : ORANGE); y += 26;

    if (!status.startReachable || !status.endReachable) {
        render::DrawTextBold(font, "OUT OF REACH!", x0 + pad, y, 22, RED);
        y += 24;
        render::DrawTextSmall(font, "Choose points inside workspace.", x0 + pad, y, 18, RED);
        y += 22;
    }

    if (status.errorText) {
        render::DrawTextBold(font, status.errorText, x0 + pad, y, 20, RED);
    }

    return paused;
}

} // namespace ui
