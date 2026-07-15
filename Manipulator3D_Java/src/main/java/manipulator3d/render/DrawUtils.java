package manipulator3d.render;

import static com.raylib.Raylib.*;
import com.raylib.Raylib.*;
import com.raylib.Colors;

/** Rendering helpers for a simple machined-looking manipulator visualization. */
public final class DrawUtils {

    private DrawUtils() {}

    private static void drawTextExAt(Font font, String text, int x, int y, float fontSize, Color color) {
        DrawTextEx(font, text, new Vector2().x(x).y(y), fontSize, 1.0f, color);
    }

    public static void drawTextBold(Font font, String text, int x, int y, float fontSize, Color color) {
        drawTextExAt(font, text, x, y, fontSize, color);
        drawTextExAt(font, text, x + 1, y, fontSize, color);
        drawTextExAt(font, text, x, y + 1, fontSize, color);
        drawTextExAt(font, text, x + 1, y + 1, fontSize, color);
    }

    public static void drawTextSmall(Font font, String text, int x, int y, float fontSize, Color color) {
        drawTextExAt(font, text, x, y, fontSize, color);
    }

    public static void drawRobotBasePedestal(Vector3 origin) {
        // Pedestal below ground slightly + base column
        Vector3 a = new Vector3().x(origin.x()).y(origin.y()).z(-0.25f);
        Vector3 b = new Vector3().x(origin.x()).y(origin.y()).z( 0.00f);
        DrawCylinderEx(a, b, 0.55f, 0.55f, 24, rgba(70,70,75,255));

        Vector3 c = new Vector3().x(origin.x()).y(origin.y()).z(0.00f);
        Vector3 d = new Vector3().x(origin.x()).y(origin.y()).z(0.35f);
        DrawCylinderEx(c, d, 0.38f, 0.34f, 24, rgba(95,95,100,255));

        // Base flange
        DrawCylinderEx(
                new Vector3().x(origin.x()).y(origin.y()).z(0.00f),
                new Vector3().x(origin.x()).y(origin.y()).z(0.06f),
                0.48f, 0.48f, 24,
                rgba(110,110,115,255)
        );
    }

    public static void drawRobotJointHousing(Vector3 center, float radius) {
        DrawSphere(center, radius, rgba(120,120,125,255));
        DrawSphereWires(center, radius, 12, 12, rgba(200,200,200,60));
        DrawCylinderEx(
                new Vector3().x(center.x()).y(center.y()).z(center.z() - 0.10f),
                new Vector3().x(center.x()).y(center.y()).z(center.z() + 0.10f),
                radius * 0.55f, radius * 0.55f, 18,
                rgba(85,85,90,255)
        );
    }

    public static void drawTaperedLink(Vector3 a, Vector3 b, float rA, float rB, Color color) {
        DrawCylinderEx(a, b, rA, rB, 20, color);
        DrawSphere(a, rA * 0.95f, rgba(140,140,145,255));
        DrawSphere(b, rB * 0.95f, rgba(140,140,145,255));
    }

    public static void drawSuctionTool(Vector3 ee, Vector3 approachDir) {
        Vector3 tip = new Vector3()
                .x(ee.x() + approachDir.x() * 0.28f)
                .y(ee.y() + approachDir.y() * 0.28f)
                .z(ee.z() + approachDir.z() * 0.28f);

        DrawCylinderEx(ee, tip, 0.06f, 0.05f, 18, rgba(40,40,45,255));

        Vector3 cupA = new Vector3().x(tip.x()).y(tip.y()).z(tip.z());
        Vector3 cupB = new Vector3()
                .x(tip.x() + approachDir.x() * 0.06f)
                .y(tip.y() + approachDir.y() * 0.06f)
                .z(tip.z() + approachDir.z() * 0.06f);

        DrawCylinderEx(cupA, cupB, 0.11f, 0.11f, 24, rgba(25,25,28,255));
        DrawSphere(tip, 0.035f, rgba(80,80,85,255));
    }

    public static Color rgba(int r, int g, int b, int a) {
        return new Color().r((byte)r).g((byte)g).b((byte)b).a((byte)a);
    }
}
