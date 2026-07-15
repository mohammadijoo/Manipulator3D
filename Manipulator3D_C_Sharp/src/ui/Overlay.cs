using System;
using System.Globalization;
using System.Numerics;
using Raylib_cs;
using Manipulator3D.robot;
using Manipulator3D.render;

namespace Manipulator3D.ui;

public struct OverlayStatus
{
    public bool startReachable;
    public bool endReachable;
    public string? errorText;
    public string phaseText;
}

public sealed class OverlayUiState
{
    // Text fields: START(x,y,z), GOAL(x,y,z)
    private readonly string[] _fields = new string[6];
    private int _activeIndex = -1;

    public OverlayUiState(Vector3 initialStart, Vector3 initialGoal)
    {
        _fields[0] = initialStart.X.ToString(CultureInfo.InvariantCulture);
        _fields[1] = initialStart.Y.ToString(CultureInfo.InvariantCulture);
        _fields[2] = initialStart.Z.ToString(CultureInfo.InvariantCulture);
        _fields[3] = initialGoal.X.ToString(CultureInfo.InvariantCulture);
        _fields[4] = initialGoal.Y.ToString(CultureInfo.InvariantCulture);
        _fields[5] = initialGoal.Z.ToString(CultureInfo.InvariantCulture);
    }

    public void SyncFromAccepted(Vector3 start, Vector3 goal)
    {
        _fields[0] = start.X.ToString(CultureInfo.InvariantCulture);
        _fields[1] = start.Y.ToString(CultureInfo.InvariantCulture);
        _fields[2] = start.Z.ToString(CultureInfo.InvariantCulture);
        _fields[3] = goal.X.ToString(CultureInfo.InvariantCulture);
        _fields[4] = goal.Y.ToString(CultureInfo.InvariantCulture);
        _fields[5] = goal.Z.ToString(CultureInfo.InvariantCulture);
    }

    public bool TryGetPendingTargets(out Vector3 start, out Vector3 goal, out string? error)
    {
        error = null;
        start = default;
        goal = default;

        if (!TryParseFloat(_fields[0], out float sx) ||
            !TryParseFloat(_fields[1], out float sy) ||
            !TryParseFloat(_fields[2], out float sz) ||
            !TryParseFloat(_fields[3], out float gx) ||
            !TryParseFloat(_fields[4], out float gy) ||
            !TryParseFloat(_fields[5], out float gz))
        {
            error = "Invalid number format. \nUse decimals like 1.25 and a dot '.'";
            return false;
        }

        if (sz < 0 || gz < 0)
        {
            error = "Invalid input: z must be >= 0";
            return false;
        }

        start = new Vector3(sx, sy, sz);
        goal = new Vector3(gx, gy, gz);
        return true;
    }

    public void UpdateTextInput()
    {
        if (_activeIndex < 0 || _activeIndex >= _fields.Length) return;

        // Character input
        int c = Raylib.GetCharPressed();
        while (c > 0)
        {
            char ch = (char)c;
            if (IsAllowedChar(ch))
                _fields[_activeIndex] += ch;

            c = Raylib.GetCharPressed();
        }

        // Backspace
        if (Raylib.IsKeyPressed(RaylibCompat.KeyBackspace))
        {
            string s = _fields[_activeIndex];
            if (s.Length > 0)
                _fields[_activeIndex] = s[..^1];
        }

        // TAB cycles fields
        if (Raylib.IsKeyPressed(RaylibCompat.KeyTab))
        {
            _activeIndex = (_activeIndex + 1) % _fields.Length;
        }

        // ESC deactivates
        if (Raylib.IsKeyPressed(RaylibCompat.KeyEscape))
        {
            _activeIndex = -1;
        }
    }

    public void SetActiveField(int idx)
    {
        _activeIndex = idx < 0 ? -1 : idx;
    }

    public int ActiveFieldIndex => _activeIndex;

    public string GetField(int idx) => _fields[idx];

    private static bool TryParseFloat(string s, out float v)
        => float.TryParse(s.Trim(), NumberStyles.Float, CultureInfo.InvariantCulture, out v);

    private static bool IsAllowedChar(char ch)
        => char.IsDigit(ch) || ch == '-' || ch == '+' || ch == '.' || ch == 'e' || ch == 'E';
}

public struct OverlayResult
{
    public bool playClicked;
    public bool pauseClicked;
}

public static class Overlay
{
    private static bool PointInRect(Vector2 p, Rectangle r)
        => Raylib.CheckCollisionPointRec(p, r);

    public static OverlayResult DrawOverlayPanel(
        Font font,
        RobotArm arm,
        Vector3 acceptedStart,
        Vector3 acceptedGoal,
        Vector3? pendingStart,
        Vector3? pendingGoal,
        OverlayStatus status,
        bool paused,
        OverlayUiState uiState,
        int screenW,
        int screenH)
    {
        OverlayResult result = new();

        const int pad = 12;
        const int x0 = 14;
        const int y0 = 14;
        const int w = 360;
        // Increase panel height; also keep it within the window height.
        int h = Math.Min(520, screenH - 28);
        if (h < 420) h = 420;

        Raylib.DrawRectangle(x0, y0, w, h, new Color(18, 18, 18, 230));
        Raylib.DrawRectangleLines(x0, y0, w, h, new Color(200, 200, 200, 255));

        int y = y0 + pad;

        DrawUtils.DrawTextBold(font, "3-DOF 2-Link Arm", x0 + pad, y, 24, Color.RayWhite);
        y += 30;

        if (!string.IsNullOrWhiteSpace(status.phaseText))
        {
            DrawUtils.DrawTextSmall(font, status.phaseText, x0 + pad, y, 18, Color.SkyBlue);
            y += 24;
        }

        // PLAY / PAUSE button
        Rectangle btn = new(x0 + pad, y, w - 2 * pad, 36);
        Color btnBg = paused ? new Color(60, 120, 60, 220) : new Color(120, 60, 60, 220);
        Raylib.DrawRectangleRounded(btn, 0.18f, 8, btnBg);
        Raylib.DrawRectangleRoundedLines(btn, 0.18f, 8, new Color(230, 230, 230, 255));


        string label = paused ? "PLAY" : "PAUSE";
        DrawUtils.DrawTextBold(font, label, x0 + pad + 10, y + 7, 22, Color.RayWhite);

        Vector2 mp = Raylib.GetMousePosition();
        if (Raylib.IsMouseButtonPressed(MouseButton.Left) && PointInRect(mp, btn))
        {
            if (paused) result.playClicked = true;
            else result.pauseClicked = true;
        }

        y += 48;

        // Input boxes
        DrawUtils.DrawTextBold(font, "START (x, y, z)", x0 + pad, y, 20, Color.RayWhite);
        y += 26;
        int startRowY = y;
        DrawVec3Inputs(font, uiState, baseX: x0 + pad, baseY: startRowY, fieldIndex0: 0, statusColor: status.startReachable ? Color.Green : Color.Orange);
        y += 58;

        DrawUtils.DrawTextBold(font, "GOAL (x, y, z)", x0 + pad, y, 20, Color.RayWhite);
        y += 26;
        int goalRowY = y;
        DrawVec3Inputs(font, uiState, baseX: x0 + pad, baseY: goalRowY, fieldIndex0: 3, statusColor: status.endReachable ? Color.Green : Color.Orange);
        y += 62;

        HandleFieldFocusByMouse(uiState, baseX: x0 + pad, field0Y: startRowY, field3Y: goalRowY);

        // While paused, Enter starts
        if (paused && Raylib.IsKeyPressed(RaylibCompat.KeyEnter))
            result.playClicked = true;

        // Link + workspace info
        var L1 = arm.Link1();
        var L2 = arm.Link2();

        string line;
        line = $"Link1 length: {L1.length_m:0.###}m"; DrawUtils.DrawTextSmall(font, line, x0 + pad, y, 18, Color.RayWhite); y += 20;
        line = $"Link1 mass  : {L1.mass_kg:0.###}kg"; DrawUtils.DrawTextSmall(font, line, x0 + pad, y, 18, Color.RayWhite); y += 20;
        line = $"Link1 inertia (joint): {L1.inertia_joint:0.#####}"; DrawUtils.DrawTextSmall(font, line, x0 + pad, y, 18, Color.RayWhite); y += 24;

        line = $"Link2 length: {L2.length_m:0.###}m"; DrawUtils.DrawTextSmall(font, line, x0 + pad, y, 18, Color.RayWhite); y += 20;
        line = $"Link2 mass  : {L2.mass_kg:0.###}kg"; DrawUtils.DrawTextSmall(font, line, x0 + pad, y, 18, Color.RayWhite); y += 20;
        line = $"Link2 inertia (joint): {L2.inertia_joint:0.#####}"; DrawUtils.DrawTextSmall(font, line, x0 + pad, y, 18, Color.RayWhite); y += 24;

        float rmin = arm.MinReach();
        float rmax = arm.MaxReach();
        line = $"Workspace |p|: [{rmin:0.##}, {rmax:0.##}]m";
        DrawUtils.DrawTextSmall(font, line, x0 + pad, y, 18, new Color(140, 200, 255, 255));
        y += 24;

        if (!status.startReachable || !status.endReachable)
        {
            DrawUtils.DrawTextBold(font, "OUT OF REACH!", x0 + pad, y, 22, Color.Red);
            y += 22;
            DrawUtils.DrawTextSmall(font, "Choose points inside workspace.", x0 + pad, y, 18, Color.Red);
            y += 20;
        }

        if (!string.IsNullOrWhiteSpace(status.errorText))
        {
            DrawUtils.DrawTextBold(font, status.errorText!, x0 + pad, y, 18, Color.Red);
        }

        uiState.UpdateTextInput();
        return result;
    }

    private static void DrawVec3Inputs(Font font, OverlayUiState uiState, int baseX, int baseY, int fieldIndex0, Color statusColor)
    {
        const int boxW = 100;
        const int boxH = 30;
        const int gap = 10;

        DrawLabeledField(font, uiState, fieldIndex0 + 0, baseX + (boxW + gap) * 0, baseY, boxW, boxH, "x", statusColor);
        DrawLabeledField(font, uiState, fieldIndex0 + 1, baseX + (boxW + gap) * 1, baseY, boxW, boxH, "y", statusColor);
        DrawLabeledField(font, uiState, fieldIndex0 + 2, baseX + (boxW + gap) * 2, baseY, boxW, boxH, "z", statusColor);

        DrawUtils.DrawTextSmall(font, "Tip: click a field, type value, TAB to next", baseX, baseY + 36, 16, new Color(200, 200, 200, 210));
    }

    private static void DrawLabeledField(Font font, OverlayUiState uiState, int idx, int x, int y, int w, int h, string label, Color borderColor)
    {
        Rectangle r = new(x, y, w, h);
        bool active = uiState.ActiveFieldIndex == idx;

        Color bg = active ? new Color(35, 35, 40, 245) : new Color(25, 25, 28, 235);
        Raylib.DrawRectangleRec(r, bg);

        Color line = active ? Color.Yellow : borderColor;
        Raylib.DrawRectangleLines(x, y, w, h, line);

        DrawUtils.DrawTextSmall(font, label, x + 6, y - 18, 16, new Color(200, 200, 200, 220));
        DrawUtils.DrawTextSmall(font, uiState.GetField(idx), x + 8, y + 7, 18, Color.RayWhite);

        if (active)
        {
            Vector2 sz = Raylib.MeasureTextEx(font, uiState.GetField(idx), 18, 1.0f);
            int textW = (int)sz.X;
            Raylib.DrawRectangle(x + 8 + textW + 2, y + 8, 2, 18, new Color(240, 240, 240, 200));
        }
    }

    private static void HandleFieldFocusByMouse(OverlayUiState uiState, int baseX, int field0Y, int field3Y)
    {
        const int boxW = 100;
        const int boxH = 30;
        const int gap = 10;

        Vector2 mp = Raylib.GetMousePosition();
        if (!Raylib.IsMouseButtonPressed(MouseButton.Left)) return;

        // START row
        for (int i = 0; i < 3; i++)
        {
            Rectangle r = new(baseX + (boxW + gap) * i, field0Y, boxW, boxH);
            if (PointInRect(mp, r))
            {
                uiState.SetActiveField(i);
                return;
            }
        }

        // GOAL row
        for (int i = 0; i < 3; i++)
        {
            Rectangle r = new(baseX + (boxW + gap) * i, field3Y, boxW, boxH);
            if (PointInRect(mp, r))
            {
                uiState.SetActiveField(3 + i);
                return;
            }
        }

        uiState.SetActiveField(-1);
    }
}
