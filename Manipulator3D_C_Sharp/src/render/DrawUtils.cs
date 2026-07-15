using System.Numerics;
using Raylib_cs;

namespace Manipulator3D.render;

public static class DrawUtils
{
    private static void DrawTextExAt(Font font, string text, int x, int y, float fontSize, Color color)
    {
        Raylib.DrawTextEx(font, text, new Vector2(x, y), fontSize, 1.0f, color);
    }

    public static void DrawTextBold(Font font, string text, int x, int y, float fontSize, Color color)
    {
        DrawTextExAt(font, text, x, y, fontSize, color);
        DrawTextExAt(font, text, x + 1, y, fontSize, color);
        DrawTextExAt(font, text, x, y + 1, fontSize, color);
        DrawTextExAt(font, text, x + 1, y + 1, fontSize, color);
    }

    public static void DrawTextSmall(Font font, string text, int x, int y, float fontSize, Color color)
    {
        DrawTextExAt(font, text, x, y, fontSize, color);
    }

    public static void DrawRobotBasePedestal(Vector3 origin)
    {
        Vector3 a = new(origin.X, origin.Y, -0.25f);
        Vector3 b = new(origin.X, origin.Y, 0.00f);
        Raylib.DrawCylinderEx(a, b, 0.55f, 0.55f, 24, new Color(70, 70, 75, 255));

        Vector3 c = new(origin.X, origin.Y, 0.00f);
        Vector3 d = new(origin.X, origin.Y, 0.35f);
        Raylib.DrawCylinderEx(c, d, 0.38f, 0.34f, 24, new Color(95, 95, 100, 255));

        Raylib.DrawCylinderEx(
            new Vector3(origin.X, origin.Y, 0.00f),
            new Vector3(origin.X, origin.Y, 0.06f),
            0.48f,
            0.48f,
            24,
            new Color(110, 110, 115, 255)
        );
    }

    public static void DrawRobotJointHousing(Vector3 center, float radius)
    {
        Raylib.DrawSphere(center, radius, new Color(120, 120, 125, 255));
        Raylib.DrawSphereWires(center, radius, 12, 12, new Color(200, 200, 200, 60));

        Raylib.DrawCylinderEx(
            new Vector3(center.X, center.Y, center.Z - 0.10f),
            new Vector3(center.X, center.Y, center.Z + 0.10f),
            radius * 0.55f,
            radius * 0.55f,
            18,
            new Color(85, 85, 90, 255)
        );
    }

    public static void DrawTaperedLink(Vector3 a, Vector3 b, float rA, float rB, Color color)
    {
        Raylib.DrawCylinderEx(a, b, rA, rB, 20, color);

        Raylib.DrawSphere(a, rA * 0.95f, new Color(140, 140, 145, 255));
        Raylib.DrawSphere(b, rB * 0.95f, new Color(140, 140, 145, 255));
    }

    public static void DrawSuctionTool(Vector3 ee, Vector3 approachDir)
    {
        Vector3 dir = approachDir;
        float len = dir.Length();
        dir = len > 1e-6f ? dir / len : Vector3.UnitX;

        Vector3 tip = ee + dir * 0.28f;
        Raylib.DrawCylinderEx(ee, tip, 0.06f, 0.05f, 18, new Color(40, 40, 45, 255));

        Vector3 cupA = tip;
        Vector3 cupB = tip + dir * 0.06f;
        Raylib.DrawCylinderEx(cupA, cupB, 0.11f, 0.11f, 24, new Color(25, 25, 28, 255));

        Raylib.DrawSphere(tip, 0.035f, new Color(80, 80, 85, 255));
    }
}
