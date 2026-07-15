#pragma once
#include <raylib.h>

namespace render {

void DrawTextBold(Font font, const char* text, int x, int y, float fontSize, Color color);
void DrawTextSmall(Font font, const char* text, int x, int y, float fontSize, Color color);

// More "real world" robot visuals
void DrawRobotBasePedestal(Vector3 origin);
void DrawRobotJointHousing(Vector3 center, float radius);
void DrawTaperedLink(Vector3 a, Vector3 b, float rA, float rB, Color color);
void DrawSuctionTool(Vector3 ee, Vector3 approachDir);

} // namespace render
