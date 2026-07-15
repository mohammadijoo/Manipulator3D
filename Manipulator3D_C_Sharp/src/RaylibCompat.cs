using System;
using Raylib_cs;

namespace Manipulator3D;

/// <summary>
/// Compatibility layer to keep the project buildable across Raylib-cs versions
/// that may rename enum members.
/// </summary>
internal static class RaylibCompat
{
    // raylib TraceLogLevel in C: LOG_ERROR is typically 5
    internal static TraceLogLevel TraceLogError => EnumOrFallback("Error", "LOG_ERROR", (TraceLogLevel)5);

    // raylib ConfigFlags: FLAG_WINDOW_RESIZABLE=0x00000004, FLAG_MSAA_4X_HINT=0x00000020
    internal static ConfigFlags WindowResizable => EnumOrFallback("WindowResizable", "FLAG_WINDOW_RESIZABLE", (ConfigFlags)0x00000004);
    internal static ConfigFlags Msaa4xHint => EnumOrFallback("Msaa4xHint", "FLAG_MSAA_4X_HINT", (ConfigFlags)0x00000020);

    // raylib KeyboardKey (GLFW keycodes): KEY_F11=300, KEY_BACKSPACE=259, KEY_ENTER=257, KEY_TAB=258, KEY_ESCAPE=256
    internal static KeyboardKey KeyF11 => EnumOrFallback("F11", "KEY_F11", (KeyboardKey)300);
    internal static KeyboardKey KeyBackspace => EnumOrFallback("Backspace", "KEY_BACKSPACE", (KeyboardKey)259);
    internal static KeyboardKey KeyEnter => EnumOrFallback("Enter", "KEY_ENTER", (KeyboardKey)257);
    internal static KeyboardKey KeyTab => EnumOrFallback("Tab", "KEY_TAB", (KeyboardKey)258);
    internal static KeyboardKey KeyEscape => EnumOrFallback("Escape", "KEY_ESCAPE", (KeyboardKey)256);

    internal static TextureFilter TextureBilinear => EnumOrFallback("Bilinear", "TEXTURE_FILTER_BILINEAR", (TextureFilter)1);


    // raylib CameraProjection: CAMERA_PERSPECTIVE=0
    internal static CameraProjection CameraPerspective => EnumOrFallback("Perspective", "CAMERA_PERSPECTIVE", (CameraProjection)0);

    private static T EnumOrFallback<T>(string preferredName, string legacyName, T fallback) where T : struct, Enum
    {
        if (Enum.TryParse(preferredName, ignoreCase: true, out T v1)) return v1;
        if (Enum.TryParse(legacyName, ignoreCase: true, out T v2)) return v2;
        return fallback;
    }
}
