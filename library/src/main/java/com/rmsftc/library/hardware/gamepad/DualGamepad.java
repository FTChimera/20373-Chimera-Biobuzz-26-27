package com.rmsftc.library.hardware.gamepad;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.commands.Commands;
import com.qualcomm.robotcore.hardware.Gamepad;

import java.util.List;
import java.util.Objects;
import java.util.function.DoubleUnaryOperator;

public class DualGamepad {
	/** Wrapped extended controller for FTC SDK `gamepad1`. */
	public final GamepadEx gamepad1;

	/** Wrapped extended controller for FTC SDK `gamepad2`. */
	public final GamepadEx gamepad2;

	/** Creates a dual wrapper with default `GamepadEx` settings for both controllers. */
	public DualGamepad(Gamepad gamepad1, Gamepad gamepad2) {
		this(new GamepadEx(gamepad1), new GamepadEx(gamepad2));
	}

	/** Creates a dual wrapper with the same custom settings applied to both controllers. */
	public DualGamepad(Gamepad gamepad1, Gamepad gamepad2, double deadzone, DoubleUnaryOperator curve, double triggerThreshold) {
		this(
				new GamepadEx(gamepad1, deadzone, curve, triggerThreshold),
				new GamepadEx(gamepad2, deadzone, curve, triggerThreshold)
		);
	}

	/** Creates a dual wrapper from two already-configured `GamepadEx` objects. */
	public DualGamepad(GamepadEx gamepad1, GamepadEx gamepad2) {
		this.gamepad1 = Objects.requireNonNull(gamepad1, "gamepad1 cannot be null");
		this.gamepad2 = Objects.requireNonNull(gamepad2, "gamepad2 cannot be null");
	}

	// ==================== UPDATE / IVY ====================

	/** Updates both wrapped gamepads. Call once per loop before reading values. */
	public void update() {
		gamepad1.update();
		gamepad2.update();
	}

	/** Ivy command that continuously updates both wrapped gamepads. */
	public Command updateCommand() {
		return Commands.infinite(this::update);
	}

	// ==================== DIRECT ACCESS ====================

	public GamepadEx getGamepad1() { return gamepad1; }

	public GamepadEx getGamepad2() { return gamepad2; }

	public Gamepad getRawGamepad1() { return gamepad1.getRawGamepad(); }

	public Gamepad getRawGamepad2() { return gamepad2.getRawGamepad(); }

	public GamepadType getType1() { return gamepad1.getType(); }

	public GamepadType getType2() { return gamepad2.getType(); }

	// ==================== BUTTONS ====================

	public boolean isPressed1(Button button) { return gamepad1.isPressed(button); }

	public boolean isPressed2(Button button) { return gamepad2.isPressed(button); }

	public boolean wasJustPressed1(Button button) { return gamepad1.wasJustPressed(button); }

	public boolean wasJustPressed2(Button button) { return gamepad2.wasJustPressed(button); }

	public boolean wasJustReleased1(Button button) { return gamepad1.wasJustReleased(button); }

	public boolean wasJustReleased2(Button button) { return gamepad2.wasJustReleased(button); }

	public boolean getToggle1(Button button) { return gamepad1.getToggle(button); }

	public boolean getToggle2(Button button) { return gamepad2.getToggle(button); }

	public void setToggle1(Button button, boolean state) { gamepad1.setToggle(button, state); }

	public void setToggle2(Button button, boolean state) { gamepad2.setToggle(button, state); }

	public Buttons getButtons1() { return gamepad1.buttons; }

	public Buttons getButtons2() { return gamepad2.buttons; }

	// ==================== STICKS ====================

	/** Returns the wrapped left-stick object for gamepad 1. */
	public Stick leftStick1() { return gamepad1.leftStick; }

	/** Returns the wrapped left-stick object for gamepad 2. */
	public Stick leftStick2() { return gamepad2.leftStick; }

	/** Returns the wrapped right-stick object for gamepad 1. */
	public Stick rightStick1() { return gamepad1.rightStick; }

	/** Returns the wrapped right-stick object for gamepad 2. */
	public Stick rightStick2() { return gamepad2.rightStick; }

	// ==================== TRIGGERS ====================

	public Trigger leftTrigger1() { return gamepad1.leftTrigger; }

	public Trigger leftTrigger2() { return gamepad2.leftTrigger; }

	public Trigger rightTrigger1() { return gamepad1.rightTrigger; }

	public Trigger rightTrigger2() { return gamepad2.rightTrigger; }

	// ==================== TOUCHPAD ====================

	public Touchpad getTouchpad1() { return gamepad1.touchpad; }

	public Touchpad getTouchpad2() { return gamepad2.touchpad; }

	public List<Touchpad.TouchPoint> getTouchpadFingers1() { return gamepad1.getTouchpadFingers(); }

	public List<Touchpad.TouchPoint> getTouchpadFingers2() { return gamepad2.getTouchpadFingers(); }

	public Touchpad.TouchPoint getTouchpad1Finger1() { return gamepad1.touchpad.getFinger1(); }

	public Touchpad.TouchPoint getTouchpad1Finger2() { return gamepad1.touchpad.getFinger2(); }

	public Touchpad.TouchPoint getTouchpad2Finger1() { return gamepad2.touchpad.getFinger1(); }

	public Touchpad.TouchPoint getTouchpad2Finger2() { return gamepad2.touchpad.getFinger2(); }

	public boolean isTouchpad1Finger1Down() { return gamepad1.touchpad.isFinger1Down(); }

	public boolean isTouchpad1Finger2Down() { return gamepad1.touchpad.isFinger2Down(); }

	public boolean isTouchpad2Finger1Down() { return gamepad2.touchpad.isFinger1Down(); }

	public boolean isTouchpad2Finger2Down() { return gamepad2.touchpad.isFinger2Down(); }

	public boolean isTouchpad1Touched() { return gamepad1.touchpad.isTouched(); }

	public boolean isTouchpad2Touched() { return gamepad2.touchpad.isTouched(); }

	public int getTouchpad1FingerCount() { return gamepad1.touchpad.getFingerCount(); }

	public int getTouchpad2FingerCount() { return gamepad2.touchpad.getFingerCount(); }

	public boolean wasTouchpad1Finger1JustPressed() { return gamepad1.touchpad.wasFinger1JustPressed(); }

	public boolean wasTouchpad1Finger1JustReleased() { return gamepad1.touchpad.wasFinger1JustReleased(); }

	public boolean wasTouchpad1Finger2JustPressed() { return gamepad1.touchpad.wasFinger2JustPressed(); }

	public boolean wasTouchpad1Finger2JustReleased() { return gamepad1.touchpad.wasFinger2JustReleased(); }

	public boolean wasTouchpad2Finger1JustPressed() { return gamepad2.touchpad.wasFinger1JustPressed(); }

	public boolean wasTouchpad2Finger1JustReleased() { return gamepad2.touchpad.wasFinger1JustReleased(); }

	public boolean wasTouchpad2Finger2JustPressed() { return gamepad2.touchpad.wasFinger2JustPressed(); }

	public boolean wasTouchpad2Finger2JustReleased() { return gamepad2.touchpad.wasFinger2JustReleased(); }

	public boolean isTouchpad1ButtonPressed() { return gamepad1.touchpad.isButtonPressed(); }

	public boolean isTouchpad2ButtonPressed() { return gamepad2.touchpad.isButtonPressed(); }

	public boolean wasTouchpad1ButtonJustPressed() { return gamepad1.touchpad.wasButtonJustPressed(); }

	public boolean wasTouchpad2ButtonJustPressed() { return gamepad2.touchpad.wasButtonJustPressed(); }

	public boolean wasTouchpad1ButtonJustReleased() { return gamepad1.touchpad.wasButtonJustReleased(); }

	public boolean wasTouchpad2ButtonJustReleased() { return gamepad2.touchpad.wasButtonJustReleased(); }

	public boolean getTouchpad1ButtonToggle() { return gamepad1.touchpad.getButtonToggle(); }

	public boolean getTouchpad2ButtonToggle() { return gamepad2.touchpad.getButtonToggle(); }

	public void setTouchpad1ButtonToggle(boolean state) { gamepad1.touchpad.setButtonToggle(state); }

	public void setTouchpad2ButtonToggle(boolean state) { gamepad2.touchpad.setButtonToggle(state); }

	// ==================== SHARED CONFIGURATION ====================

	/** Applies the same joystick deadzone to both wrapped gamepads. */
	public DualGamepad setJoystickDeadzone(double deadzone) {
		gamepad1.setJoystickDeadzone(deadzone);
		gamepad2.setJoystickDeadzone(deadzone);
		return this;
	}

	/** Applies the same stick curve to both wrapped gamepads. */
	public DualGamepad setStickCurve(DoubleUnaryOperator curve) {
		gamepad1.setStickCurve(curve);
		gamepad2.setStickCurve(curve);
		return this;
	}

	/** Applies the same trigger threshold to both wrapped gamepads. */
	public DualGamepad setTriggerThreshold(double threshold) {
		gamepad1.setTriggerThreshold(threshold);
		gamepad2.setTriggerThreshold(threshold);
		return this;
	}

	// ==================== LED LIGHT BAR ====================

	public void setLedColor1(double red, double green, double blue, int durationMs) {
		gamepad1.setLedColor(red, green, blue, durationMs);
	}

	public void setLedColor2(double red, double green, double blue, int durationMs) {
		gamepad2.setLedColor(red, green, blue, durationMs);
	}

	public void setLedColor(double red, double green, double blue, int durationMs) {
		setLedColor1(red, green, blue, durationMs);
		setLedColor2(red, green, blue, durationMs);
	}

	public void setLedColor1(double red, double green, double blue) {
		gamepad1.setLedColor(red, green, blue);
	}

	public void setLedColor2(double red, double green, double blue) {
		gamepad2.setLedColor(red, green, blue);
	}

	public void setLedColor(double red, double green, double blue) {
		setLedColor1(red, green, blue);
		setLedColor2(red, green, blue);
	}

	public void ledOff1() { gamepad1.ledOff(); }

	public void ledOff2() { gamepad2.ledOff(); }

	public void ledOff() {
		ledOff1();
		ledOff2();
	}

	// ==================== RUMBLE ====================

	public void rumble1(int durationMs) { gamepad1.rumble(durationMs); }

	public void rumble2(int durationMs) { gamepad2.rumble(durationMs); }

	public void rumble(int durationMs) {
		rumble1(durationMs);
		rumble2(durationMs);
	}

	public void rumble1(double left, double right, int durationMs) {
		gamepad1.rumble(left, right, durationMs);
	}

	public void rumble2(double left, double right, int durationMs) {
		gamepad2.rumble(left, right, durationMs);
	}

	public void rumble(double left, double right, int durationMs) {
		rumble1(left, right, durationMs);
		rumble2(left, right, durationMs);
	}

	public void rumbleContinuous1(double left, double right) {
		gamepad1.rumbleContinuous(left, right);
	}

	public void rumbleContinuous2(double left, double right) {
		gamepad2.rumbleContinuous(left, right);
	}

	public void rumbleContinuous(double left, double right) {
		rumbleContinuous1(left, right);
		rumbleContinuous2(left, right);
	}

	public void rumbleBlips1(int count) { gamepad1.rumbleBlips(count); }

	public void rumbleBlips2(int count) { gamepad2.rumbleBlips(count); }

	public void rumbleBlips(int count) {
		rumbleBlips1(count);
		rumbleBlips2(count);
	}

	public void runRumbleEffect1(Gamepad.RumbleEffect effect) { gamepad1.runRumbleEffect(effect); }

	public void runRumbleEffect2(Gamepad.RumbleEffect effect) { gamepad2.runRumbleEffect(effect); }

	public void runRumbleEffect(Gamepad.RumbleEffect effect) {
		runRumbleEffect1(effect);
		runRumbleEffect2(effect);
	}

	public void stopRumble1() { gamepad1.stopRumble(); }

	public void stopRumble2() { gamepad2.stopRumble(); }

	public void stopRumble() {
		stopRumble1();
		stopRumble2();
	}

	public boolean isRumbling1() { return gamepad1.isRumbling(); }

	public boolean isRumbling2() { return gamepad2.isRumbling(); }

	public boolean isEitherRumbling() { return isRumbling1() || isRumbling2(); }

	// ==================== ACTIVITY ====================

	/** @return true if gamepad 1 currently has any active input. */
	public boolean isGamepad1Active() { return gamepad1.anyInput(); }

	/** @return true if gamepad 2 currently has any active input. */
	public boolean isGamepad2Active() { return gamepad2.anyInput(); }

	/** @return true if either wrapped gamepad currently has any active input. */
	public boolean isEitherGamepadActive() { return isGamepad1Active() || isGamepad2Active(); }

	public boolean anyInput1() { return isGamepad1Active(); }

	public boolean anyInput2() { return isGamepad2Active(); }

	public boolean anyInput() { return isEitherGamepadActive(); }
}
