#include "render/DrawUtils.h"
#include <raymath.h>

namespace render {

static void DrawTextExAt(Font font, const char* text, int x, int y, float fontSize, Color color) {
    DrawTextEx(font, text, Vector2{(float)x, (float)y}, fontSize, 1.0f, color);
}

void DrawTextBold(Font font, const char* text, int x, int y, float fontSize, Color color) {
    DrawTextExAt(font, text, x, y, fontSize, color);
    DrawTextExAt(font, text, x+1, y, fontSize, color);
    DrawTextExAt(font, text, x, y+1, fontSize, color);
    DrawTextExAt(font, text, x+1, y+1, fontSize, color);
}

void DrawTextSmall(Font font, const char* text, int x, int y, float fontSize, Color color) {
    DrawTextExAt(font, text, x, y, fontSize, color);
}

void DrawRobotBasePedestal(Vector3 origin) {
    // Pedestal below ground slightly + base column
    Vector3 a = {origin.x, origin.y, -0.25f};
    Vector3 b = {origin.x, origin.y,  0.00f};
    DrawCylinderEx(a, b, 0.55f, 0.55f, 24, Color{70,70,75,255});

    Vector3 c = {origin.x, origin.y, 0.00f};
    Vector3 d = {origin.x, origin.y, 0.35f};
    DrawCylinderEx(c, d, 0.38f, 0.34f, 24, Color{95,95,100,255});

    // Base flange
    DrawCylinderEx({origin.x, origin.y, 0.00f}, {origin.x, origin.y, 0.06f}, 0.48f, 0.48f, 24, Color{110,110,115,255});
}

void DrawRobotJointHousing(Vector3 center, float radius) {
    // Joint housing as sphere + small collar
    DrawSphere(center, radius, Color{120,120,125,255});
    DrawSphereWires(center, radius, 12, 12, Color{200,200,200,60});
    DrawCylinderEx(
        {center.x, center.y, center.z - 0.10f},
        {center.x, center.y, center.z + 0.10f},
        radius * 0.55f, radius * 0.55f, 18,
        Color{85,85,90,255}
    );
}

void DrawTaperedLink(Vector3 a, Vector3 b, float rA, float rB, Color color) {
    // Link body
    DrawCylinderEx(a, b, rA, rB, 20, color);

    // End caps to look more machined
    DrawSphere(a, rA * 0.95f, Color{140,140,145,255});
    DrawSphere(b, rB * 0.95f, Color{140,140,145,255});
}

void DrawSuctionTool(Vector3 ee, Vector3 approachDir) {
    // Suction cup extends forward along approachDir
    Vector3 tip = Vector3Add(ee, Vector3Scale(approachDir, 0.28f));
    DrawCylinderEx(ee, tip, 0.06f, 0.05f, 18, Color{40,40,45,255});

    // Cup (wider disk-ish) using short cylinder
    Vector3 cupA = Vector3Add(tip, Vector3Scale(approachDir, 0.00f));
    Vector3 cupB = Vector3Add(tip, Vector3Scale(approachDir, 0.06f));
    DrawCylinderEx(cupA, cupB, 0.11f, 0.11f, 24, Color{25,25,28,255});

    // Small highlight
    DrawSphere(tip, 0.035f, Color{80,80,85,255});
}

} // namespace render
