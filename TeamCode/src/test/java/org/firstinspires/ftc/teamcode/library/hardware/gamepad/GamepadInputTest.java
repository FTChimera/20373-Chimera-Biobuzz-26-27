package org.firstinspires.ftc.teamcode.library.hardware.gamepad;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.library.math.MathUtils;
import org.junit.Test;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertTrue;

public class GamepadInputTest {
    @Test
    public void readersButtonsAndTypesTrackEdgesTogglesAndCapabilities() {
        Gamepad gamepad = GamepadTestUtils.newGamepad(Gamepad.Type.SONY_PS4);
        Reader reader = new Reader(Button.A);
        Buttons buttons = new Buttons();

        reader.update(gamepad);
        buttons.update(gamepad);
        assertFalse(reader.isPressed());
        assertEquals(GamepadType.PS4, GamepadType.fromGamepad(gamepad));
        assertEquals(GamepadType.UNKNOWN, GamepadType.fromGamepad(null));
        assertTrue(GamepadType.PS4.hasTouchpad());
        assertTrue(GamepadType.PS4.hasLightBar());
        assertTrue(GamepadType.PS4.hasRumble());

        gamepad.a = true;
        gamepad.x = true;
        reader.update(gamepad);
        buttons.update(gamepad);

        assertTrue(reader.isPressed());
        assertTrue(reader.wasJustPressed());
        assertTrue(reader.getToggle());
        assertTrue(buttons.wasJustPressed(Button.A));
        assertTrue(buttons.wasJustPressed(Button.X));
        assertTrue(buttons.anyPressed());
        assertTrue(buttons.anyJustPressed());
        for (Button button : Button.values()) {
            assertSame(buttons.getReader(button), buttons.getReaders().get(button));
        }
        assertSame(buttons.getReader(Button.A), buttons.getReader(Button.A.copy()));

        gamepad.a = false;
        gamepad.x = false;
        reader.update(gamepad);
        buttons.update(gamepad);
        assertTrue(reader.wasJustReleased());
        assertTrue(buttons.wasJustReleased(Button.A));
    }

    @Test
    public void stickAndTriggerApplyCurvesDeadzonesAndThresholds() {
        Gamepad gamepad = GamepadTestUtils.newGamepad(Gamepad.Type.LOGITECH_F310);
        Stick stick = Stick.left(0.1, value -> Math.copySign(value * value, value));
        Trigger trigger = new Trigger(Trigger.Side.RIGHT, 2.0);

        gamepad.left_stick_x = 0.05f;
        gamepad.left_stick_y = -0.4f;
        gamepad.left_stick_button = true;
        stick.update(gamepad);

        assertEquals(0.0, stick.getX(), MathUtils.EPSILON);
        assertEquals(0.16, stick.getY(), MathUtils.EPSILON);
        assertEquals(0.4, stick.getRawY(), MathUtils.EPSILON);
        assertTrue(stick.isButtonPressed());
        assertTrue(stick.wasButtonJustPressed());

        gamepad.left_stick_x = 0.5f;
        gamepad.left_stick_y = 0.0f;
        gamepad.left_stick_button = false;
        stick.update(gamepad);

        assertEquals(0.25, stick.getX(), MathUtils.EPSILON);
        assertEquals(0.0, stick.getY(), MathUtils.EPSILON);
        assertEquals(0.45, stick.getDeltaX(), 1e-6);
        assertTrue(stick.wasButtonJustReleased());

        assertEquals(1.0, trigger.getThreshold(), MathUtils.EPSILON);
        gamepad.right_trigger = 0.3f;
        trigger.update(gamepad);
        assertFalse(trigger.isPressed());

        gamepad.right_trigger = 1.0f;
        trigger.update(gamepad);
        assertTrue(trigger.isPressed());
        assertTrue(trigger.wasJustPressed());
        assertTrue(trigger.getToggle());

        trigger.setOutputTransform(value -> Math.min(1.0, value * 2.0));
        gamepad.right_trigger = 0.4f;
        trigger.update(gamepad);
        assertEquals(0.4, trigger.getRawValue(), MathUtils.EPSILON);
        assertEquals(0.8, trigger.getValue(), MathUtils.EPSILON);
        assertFalse(trigger.isPressed());

        gamepad.right_trigger = 0.6f;
        trigger.update(gamepad);
        assertEquals(1.0, trigger.getValue(), MathUtils.EPSILON);
        assertFalse(trigger.wasJustPressed());
    }

    @Test
    public void touchpadTracksFingersAndButtonTransitions() {
        Gamepad gamepad = GamepadTestUtils.newGamepad(Gamepad.Type.SONY_PS4);
        Touchpad touchpad = new Touchpad();

        gamepad.touchpad_finger_1 = true;
        gamepad.touchpad_finger_1_x = 0.25f;
        gamepad.touchpad_finger_1_y = -0.5f;
        gamepad.touchpad = true;
        touchpad.update(gamepad);

        assertTrue(touchpad.isTouched());
        assertEquals(1, touchpad.getFingerCount());
        assertTrue(touchpad.wasFinger1JustPressed());
        assertTrue(touchpad.wasButtonJustPressed());
        assertTrue(touchpad.getButtonToggle());
        assertEquals(0.25, touchpad.getFinger1().x, MathUtils.EPSILON);
        assertEquals(-0.5, touchpad.getFinger1().y, MathUtils.EPSILON);
        assertEquals(Math.hypot(0.25, -0.5), touchpad.getFinger1().distanceFromCenter(), MathUtils.EPSILON);

        gamepad.touchpad_finger_1 = false;
        gamepad.touchpad = false;
        touchpad.update(gamepad);

        assertTrue(touchpad.wasFinger1JustReleased());
        assertTrue(touchpad.wasButtonJustReleased());
        assertFalse(touchpad.isTouched());
    }

    @Test
    public void gamepadExAndDualGamepadPropagateUpdatesAndSharedSettings() {
        Gamepad gamepad1 = GamepadTestUtils.newGamepad(Gamepad.Type.SONY_PS4);
        Gamepad gamepad2 = GamepadTestUtils.newGamepad(Gamepad.Type.XBOX_360);
        GamepadEx extended = new GamepadEx(gamepad1);
        DualGamepad dual = new DualGamepad(gamepad1, gamepad2);

        assertSame(extended, extended.setJoystickDeadzone(0.2).setStickCurve(x -> x)
                .setTriggerOutputTransform(x -> Math.min(1.0, x * 2.0)).setTriggerThreshold(0.7));

        gamepad1.a = true;
        gamepad1.left_stick_y = -1.0f;
        extended.update();

        assertEquals(GamepadType.PS4, extended.getType());
        assertTrue(extended.wasJustPressed(Button.A));
        assertEquals(1.0, extended.leftStickY(), MathUtils.EPSILON);
        assertTrue(extended.anyInput());

        gamepad2.right_trigger = 1.0f;
        dual.setTriggerThreshold(0.6);
        dual.update();

        assertTrue(dual.isGamepad1Active());
        assertTrue(dual.isGamepad2Active());
        assertTrue(dual.anyInput());
        assertEquals(GamepadType.XBOX_360, dual.getType2());
        assertTrue(dual.rightTrigger2().isPressed());
    }

    @Test
    public void GpBindingsReusesUpdatedReadersFromWrapper() {
        Gamepad gamepad = GamepadTestUtils.newGamepad(Gamepad.Type.SONY_PS4);
        GamepadEx extended = new GamepadEx(gamepad);
        GpBindings GpBindings = extended.result(Button.A, Button.BACK);

        gamepad.a = true;
        extended.update();

        assertTrue(GpBindings.contains(Button.A));
        assertTrue(GpBindings.isPressed(Button.A));
        assertTrue(GpBindings.wasJustPressed(Button.A));
        assertFalse(GpBindings.contains(Button.Y));
    }
}
