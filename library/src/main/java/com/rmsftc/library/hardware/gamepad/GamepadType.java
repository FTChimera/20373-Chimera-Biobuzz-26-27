package com.rmsftc.library.hardware.gamepad;

import com.qualcomm.robotcore.hardware.Gamepad;

/**
 * Identifies the physical type of gamepad and exposes its capabilities.
 * Use {@link #fromGamepad(Gamepad)} to detect the type at runtime.
 *
 * <pre>
 * GamepadType type = GamepadType.fromGamepad(gamepad1);
 * if (type.hasTouchpad())  { ... }
 * if (type.hasLightBar())  { ... }
 * if (type.hasRumble())    { ... }
 * </pre>
 */
public enum GamepadType {
    //                          FTC SDK Type              touchpad  lightBar  rumble
    LOGITECH_F310(Gamepad.Type.LOGITECH_F310,             false,    false,    false),
    XBOX_360     (Gamepad.Type.XBOX_360,                  false,    false,    true),
    PS4          (Gamepad.Type.SONY_PS4,                  true,     true,     true),
    UNKNOWN      (null,                           false,    false,    false);

    private final Gamepad.Type sdkType;
    private final boolean touchpad;
    private final boolean lightBar;
    private final boolean rumble;

    GamepadType(Gamepad.Type sdkType, boolean touchpad, boolean lightBar, boolean rumble) {
        this.sdkType = sdkType;
        this.touchpad = touchpad;
        this.lightBar = lightBar;
        this.rumble = rumble;
    }

    /**
     * Detects the GamepadType from a raw FTC Gamepad.
     * @param gamepad The raw FTC Gamepad
     * @return The matching GamepadType, or {@link #UNKNOWN}
     */
    public static GamepadType fromGamepad(Gamepad gamepad) {
        if (gamepad == null || gamepad.type() == null) return UNKNOWN;
        for (GamepadType gt : values()) {
            if (gt.sdkType != null && gt.sdkType == gamepad.type()) {
                return gt;
            }
        }
        return UNKNOWN;
    }

    /** @return The underlying FTC SDK {@link Gamepad.Type}, or null for UNKNOWN */
    public Gamepad.Type getSdkType() { return sdkType; }

    /** @return true if this gamepad type has a touchpad (PS4/PS5) */
    public boolean hasTouchpad() { return touchpad; }

    /** @return true if this gamepad type has an LED light bar (PS4/PS5) */
    public boolean hasLightBar() { return lightBar; }

    /** @return true if this gamepad type supports rumble motors (Xbox, PS4/PS5) */
    public boolean hasRumble() { return rumble; }
}
