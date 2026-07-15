package manipulator3d.ui;

import manipulator3d.robot.RobotArm;
import manipulator3d.robot.Vec3f;
import manipulator3d.render.DrawUtils;

import static com.raylib.Raylib.*;
import static com.raylib.Colors.*;
import com.raylib.Raylib.*;

/**
 * HUD panel:
 * - Start/Goal text inputs (editable while paused)
 * - Play/Pause button
 * - Reachability / workspace info
 */
public final class Overlay {

    private Overlay() {}

    public enum Action {
        NONE,
        PLAY_PRESSED,
        PAUSE_PRESSED
    }

    public static final class OverlayStatus {
        public boolean startReachable = false;
        public boolean endReachable   = false;
        public String  errorText      = null;
        public String  phaseText      = "";
    }

    /** Persistent UI state (text buffers + focus). Keep one instance in Main and reuse each frame. */
    public static final class InputState {
        public final StringBuilder startText = new StringBuilder("1 2 1");
        public final StringBuilder goalText  = new StringBuilder("2 3 2");

        // 0=none, 1=start, 2=goal
        public int focus = 0;

        public void setStartDefault(Vec3f v) {
            startText.setLength(0);
            startText.append(trim3(v.x)).append(' ')
                     .append(trim3(v.y)).append(' ')
                     .append(trim3(v.z));
        }

        public void setGoalDefault(Vec3f v) {
            goalText.setLength(0);
            goalText.append(trim3(v.x)).append(' ')
                    .append(trim3(v.y)).append(' ')
                    .append(trim3(v.z));
        }

        private static String trim3(float f) {
            return String.format(java.util.Locale.US, "%.3f", f).replaceAll("0+$", "").replaceAll("\\.$", "");
        }
    }

    private static float norm3(Vec3f v) {
        return (float)Math.sqrt(v.x*v.x + v.y*v.y + v.z*v.z);
    }

    private static boolean pointInRect(Vector2 p, Rectangle r) {
        return (p.x() >= r.x() && p.x() <= r.x() + r.width() &&
                p.y() >= r.y() && p.y() <= r.y() + r.height());
    }

    private static boolean isAllowedChar(int c) {
        // Allow: digits, sign, dot, space, exponent markers
        return (c >= '0' && c <= '9') || c == '-' || c == '+' || c == '.' || c == ' ' || c == 'e' || c == 'E';
    }

    private static void handleTextEditing(StringBuilder sb, int maxLen) {
        // Typed characters
        int ch = GetCharPressed();
        while (ch > 0) {
            if (isAllowedChar(ch) && sb.length() < maxLen) {
                sb.append((char) ch);
            }
            ch = GetCharPressed();
        }

        // Backspace
        if (IsKeyPressed(KEY_BACKSPACE) && sb.length() > 0) {
            sb.deleteCharAt(sb.length() - 1);
        }

        // Ctrl+U style clear (optional convenience)
        if (IsKeyDown(KEY_LEFT_CONTROL) && IsKeyPressed(KEY_U)) {
            sb.setLength(0);
        }
    }

    /**
     * Draws the overlay. Returns a UI action event.
     *
     * - While paused: Start/Goal text boxes are editable.
     * - Clicking PLAY requests Main to validate/apply and start sim.
     * - Clicking PAUSE requests Main to freeze sim.
     */
    public static Action drawOverlayPanel(
        Font font,
        RobotArm arm,
        Vec3f start,
        Vec3f goal,
        InputState input,
        OverlayStatus status,
        boolean paused,
        int screenW,
        int screenH
    ) {
        // Panel geometry
        final int pad = 12;
        final int x0  = 14;
        final int y0  = 14;
        final int w   = 360;
        final int h   = 440;

        DrawRectangle(x0, y0, w, h, DrawUtils.rgba(18, 18, 18, 230));
        DrawRectangleLines(x0, y0, w, h, DrawUtils.rgba(200, 200, 200, 255));

        int y = y0 + pad;

        DrawUtils.drawTextBold(font, "3-DOF 2-Link Arm", x0 + pad, y, 24, RAYWHITE);
        y += 30;

        if (status.phaseText != null && !status.phaseText.isEmpty()) {
            DrawUtils.drawTextSmall(font, status.phaseText, x0 + pad, y, 18, SKYBLUE);
            y += 24;
        }

        // Play/Pause button
        Rectangle btn = new Rectangle().x(x0 + pad).y(y).width(w - 2*pad).height(34);
        Color btnBg = paused ? DrawUtils.rgba(60, 120, 60, 220) : DrawUtils.rgba(120, 60, 60, 220);
        DrawRectangleRounded(btn, 0.18f, 8, btnBg);
        DrawRectangleRoundedLines(btn, 0.18f, 8, DrawUtils.rgba(230,230,230,255));

        String label = paused ? "PLAY (apply inputs)" : "PAUSE";
        DrawUtils.drawTextBold(font, label, x0 + pad + 10, y + 6, 20, RAYWHITE);

        Vector2 mp = GetMousePosition();
        boolean btnClicked = IsMouseButtonPressed(MOUSE_BUTTON_LEFT) && pointInRect(mp, btn);

        y += 48;

        // Input fields (editable only when paused)
        DrawUtils.drawTextSmall(font, "START (x y z)  z>=0", x0 + pad, y, 16, paused ? RAYWHITE : DARKGRAY);
        y += 18;

        Rectangle startBox = new Rectangle().x(x0 + pad).y(y).width(w - 2*pad).height(28);
        Color startBg = (paused && input.focus == 1) ? DrawUtils.rgba(45,45,50,255) : DrawUtils.rgba(30,30,34,255);
        DrawRectangleRec(startBox, startBg);
        DrawRectangleLines((int)startBox.x(), (int)startBox.y(), (int)startBox.width(), (int)startBox.height(),
                (paused && input.focus == 1) ? SKYBLUE : DrawUtils.rgba(140,140,140,220));
        DrawUtils.drawTextSmall(font, input.startText.toString(), (int)startBox.x() + 8, (int)startBox.y() + 6, 18,
                paused ? RAYWHITE : DARKGRAY);

        y += 36;

        DrawUtils.drawTextSmall(font, "GOAL  (x y z)  z>=0", x0 + pad, y, 16, paused ? RAYWHITE : DARKGRAY);
        y += 18;

        Rectangle goalBox = new Rectangle().x(x0 + pad).y(y).width(w - 2*pad).height(28);
        Color goalBg = (paused && input.focus == 2) ? DrawUtils.rgba(45,45,50,255) : DrawUtils.rgba(30,30,34,255);
        DrawRectangleRec(goalBox, goalBg);
        DrawRectangleLines((int)goalBox.x(), (int)goalBox.y(), (int)goalBox.width(), (int)goalBox.height(),
                (paused && input.focus == 2) ? SKYBLUE : DrawUtils.rgba(140,140,140,220));
        DrawUtils.drawTextSmall(font, input.goalText.toString(), (int)goalBox.x() + 8, (int)goalBox.y() + 6, 18,
                paused ? RAYWHITE : DARKGRAY);

        y += 42;

        // Handle focus + editing
        if (paused && IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
            if (pointInRect(mp, startBox)) input.focus = 1;
            else if (pointInRect(mp, goalBox)) input.focus = 2;
            else if (!pointInRect(mp, btn)) input.focus = 0;
        }

        if (paused) {
            if (input.focus == 1) handleTextEditing(input.startText, 48);
            else if (input.focus == 2) handleTextEditing(input.goalText, 48);
        }

        // Link info
        var L1 = arm.link1();
        var L2 = arm.link2();

        DrawUtils.drawTextSmall(font, String.format("Link1 L=%.3fm  m=%.3fkg", L1.lengthM, L1.massKg),
                x0 + pad, y, 16, RAYWHITE); y += 18;
        DrawUtils.drawTextSmall(font, String.format("Link2 L=%.3fm  m=%.3fkg", L2.lengthM, L2.massKg),
                x0 + pad, y, 16, RAYWHITE); y += 20;

        float rmin = arm.minReach();
        float rmax = arm.maxReach();
        DrawUtils.drawTextSmall(font, String.format("Workspace |p|: [%.2f, %.2f]m", rmin, rmax),
                x0 + pad, y, 16, DrawUtils.rgba(140, 200, 255, 255));
        y += 22;

        // Current (active) start/goal summary + reachability flags
        float ds = norm3(start);
        float dg = norm3(goal);

        Color cs = status.startReachable ? GREEN : ORANGE;
        Color cg = status.endReachable   ? GREEN : ORANGE;

        DrawUtils.drawTextSmall(font, String.format("Active START |p|=%.3f  (%.2f, %.2f, %.2f)", ds, start.x, start.y, start.z),
                x0 + pad, y, 16, cs); y += 18;
        DrawUtils.drawTextSmall(font, String.format("Active GOAL  |p|=%.3f  (%.2f, %.2f, %.2f)", dg, goal.x, goal.y, goal.z),
                x0 + pad, y, 16, cg); y += 22;

        if (!status.startReachable || !status.endReachable) {
            DrawUtils.drawTextBold(font, "OUT OF REACH!", x0 + pad, y, 20, RED);
            y += 22;
            DrawUtils.drawTextSmall(font, "Choose points inside workspace.", x0 + pad, y, 16, RED);
            y += 18;
        }

        if (status.errorText != null && !status.errorText.isEmpty()) {
            DrawUtils.drawTextBold(font, status.errorText, x0 + pad, y, 18, RED);
        }

        // Emit action
        if (btnClicked) {
            return paused ? Action.PLAY_PRESSED : Action.PAUSE_PRESSED;
        }
        return Action.NONE;
    }
}
