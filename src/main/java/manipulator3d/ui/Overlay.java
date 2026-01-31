package manipulator3d.ui;

import manipulator3d.robot.RobotArm;
import manipulator3d.robot.Vec3f;
import manipulator3d.render.DrawUtils;

import static com.raylib.Raylib.*;
import static com.raylib.Colors.*;
import com.raylib.Raylib.*;

/** Simple HUD panel with link properties, workspace info, reachability and a pause button. */
public final class Overlay {

    private Overlay() {}

    public static final class OverlayStatus {
        public boolean startReachable = false;
        public boolean endReachable   = false;
        public String errorText       = null;
        public String phaseText       = "";
    }

    private static float norm3(Vec3f v) {
        return (float)Math.sqrt(v.x*v.x + v.y*v.y + v.z*v.z);
    }

    private static boolean pointInRect(Vector2 p, Rectangle r) {
        return (p.x() >= r.x() && p.x() <= r.x() + r.width() &&
                p.y() >= r.y() && p.y() <= r.y() + r.height());
    }

    /** Returns updated paused state (toggle via button click). */
    public static boolean drawOverlayPanel(
            Font font,
            RobotArm arm,
            Vec3f start,
            Vec3f goal,
            OverlayStatus status,
            boolean paused,
            int screenW,
            int screenH
    ) {
        final int pad = 12;
        final int x0  = 14;
        final int y0  = 14;
        final int w   = 320;
        final int h   = 360;

        DrawRectangle(x0, y0, w, h, DrawUtils.rgba(18, 18, 18, 230));
        DrawRectangleLines(x0, y0, w, h, DrawUtils.rgba(200, 200, 200, 255));

        int y = y0 + pad;

        DrawUtils.drawTextBold(font, "3-DOF 2-Link Arm", x0 + pad, y, 24, RAYWHITE);
        y += 30;

        if (status.phaseText != null && !status.phaseText.isEmpty()) {
            DrawUtils.drawTextSmall(font, status.phaseText, x0 + pad, y, 18, SKYBLUE);
            y += 24;
        }

        // Pause / Play button
        Rectangle btn = new Rectangle().x(x0 + pad).y(y).width(w - 2*pad).height(34);
        Color btnBg = paused ? DrawUtils.rgba(60, 120, 60, 220) : DrawUtils.rgba(120, 60, 60, 220);
        DrawRectangleRounded(btn, 0.18f, 8, btnBg);
        DrawRectangleRoundedLines(btn, 0.18f, 8, 2.0f, DrawUtils.rgba(230,230,230,255));

        String label = paused ? "PLAY" : "PAUSE";
        DrawUtils.drawTextBold(font, label, x0 + pad + 10, y + 6, 22, RAYWHITE);

        Vector2 mp = GetMousePosition();
        if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT) && pointInRect(mp, btn)) {
            paused = !paused;
        }

        y += 46;

        var L1 = arm.link1();
        var L2 = arm.link2();

        DrawUtils.drawTextSmall(font, String.format("Link1 length: %.3fm", L1.lengthM), x0 + pad, y, 18, RAYWHITE); y += 20;
        DrawUtils.drawTextSmall(font, String.format("Link1 mass  : %.3fkg", L1.massKg),  x0 + pad, y, 18, RAYWHITE); y += 20;
        DrawUtils.drawTextSmall(font, String.format("Link1 inertia (joint): %.5f", L1.inertiaJoint), x0 + pad, y, 18, RAYWHITE); y += 24;

        DrawUtils.drawTextSmall(font, String.format("Link2 length: %.3fm", L2.lengthM), x0 + pad, y, 18, RAYWHITE); y += 20;
        DrawUtils.drawTextSmall(font, String.format("Link2 mass  : %.3fkg", L2.massKg),  x0 + pad, y, 18, RAYWHITE); y += 20;
        DrawUtils.drawTextSmall(font, String.format("Link2 inertia (joint): %.5f", L2.inertiaJoint), x0 + pad, y, 18, RAYWHITE); y += 26;

        float rmin = arm.minReach();
        float rmax = arm.maxReach();
        DrawUtils.drawTextSmall(font, String.format("Workspace |p|: [%.2f, %.2f]m", rmin, rmax),
                x0 + pad, y, 18, DrawUtils.rgba(140, 200, 255, 255));
        y += 26;

        float ds = norm3(start);
        float dg = norm3(goal);

        Color cs = status.startReachable ? GREEN : ORANGE;
        Color cg = status.endReachable   ? GREEN : ORANGE;

        DrawUtils.drawTextSmall(font, String.format("Start |p|=%.3f", ds), x0 + pad, y, 18, cs); y += 20;
        DrawUtils.drawTextSmall(font, String.format("Start: (%.2f, %.2f, %.2f)", start.x, start.y, start.z), x0 + pad, y, 18, cs); y += 24;

        DrawUtils.drawTextSmall(font, String.format("Goal  |p|=%.3f", dg), x0 + pad, y, 18, cg); y += 20;
        DrawUtils.drawTextSmall(font, String.format("Goal : (%.2f, %.2f, %.2f)", goal.x, goal.y, goal.z), x0 + pad, y, 18, cg); y += 26;

        if (!status.startReachable || !status.endReachable) {
            DrawUtils.drawTextBold(font, "OUT OF REACH!", x0 + pad, y, 22, RED);
            y += 24;
            DrawUtils.drawTextSmall(font, "Choose points inside workspace.", x0 + pad, y, 18, RED);
            y += 22;
        }

        if (status.errorText != null && !status.errorText.isEmpty()) {
            DrawUtils.drawTextBold(font, status.errorText, x0 + pad, y, 20, RED);
        }

        return paused;
    }
}
