package org.firstinspires.ftc.teamcode.opmodes

import com.qualcomm.robotcore.hardware.Gamepad
import org.firstinspires.ftc.teamcode.util.hardware.gamepad.FancyButton
import org.firstinspires.ftc.teamcode.util.hardware.gamepad.GpFancyButton

class Controls(
    var gamepad1: Gamepad,
    var gamepad2: Gamepad,
    val oneController: Boolean
) {

    // Starts pressed: MainTeleOp enables smart intake at init, so the toggle
    // must reflect that or the first press is swallowed (it would "turn on"
    // what is already on) and the intake takes two presses to stop.
    private val intakeControlButton = GpFancyButton(FancyButton.PressType.Toggle) {
        secondary().x
    }.apply {
        setPressed(true)
    }

    fun intakeControl(): GpFancyButton = intakeControlButton

    private val intakeEjectButton = GpFancyButton(FancyButton.PressType.Normal) {
        secondary().back
    }

    fun intakeEject(): GpFancyButton = intakeEjectButton

    private val reZeroVisionButton = GpFancyButton(FancyButton.PressType.LongPress) {
        secondary().start
    }

    fun reZeroVision(): GpFancyButton = reZeroVisionButton

    private val moveToParkZoneButton = GpFancyButton(FancyButton.PressType.Normal) {
        primary().dpad_up
    }

    fun moveToParkZone(): GpFancyButton = moveToParkZoneButton

    private val stopParkButton = GpFancyButton(FancyButton.PressType.Normal) {
        primary().dpad_down
    }

    fun stopPark(): GpFancyButton = stopParkButton

    private val flipHiveButton = GpFancyButton(FancyButton.PressType.RisingEdge) {
        secondary().dpad_right
    }

    fun flipHive(): GpFancyButton = flipHiveButton

    // Deployable intake ramp. Starts stowed, matching Intake's init state.
    // dpad_left is free on both gamepads, so this does not collide with park
    // (primary dpad_up/down) in one-controller mode.
    private val deployerButton = GpFancyButton(FancyButton.PressType.Toggle) {
        secondary().dpad_left
    }.apply {
        setPressed(false)
    }

    fun deployer(): GpFancyButton = deployerButton

    private val runTransferFastButton = GpFancyButton(FancyButton.PressType.Normal) {
        secondary().right_bumper
    }

    fun runTransferFast(): GpFancyButton = runTransferFastButton

    private val runTransferSlowButton = GpFancyButton(FancyButton.PressType.Normal) {
        secondary().left_bumper
    }

    fun runTransferSlow(): GpFancyButton = runTransferSlowButton

    private val panicShootButton = GpFancyButton(FancyButton.PressType.Normal) {
        secondary().b
    }

    fun panicShoot(): GpFancyButton = panicShootButton

    private val autoAlignAndStartShootButton = GpFancyButton(FancyButton.PressType.Normal) {
        secondary().y
    }

    fun autoAlignAndStartShoot(): GpFancyButton = autoAlignAndStartShootButton

    private val keepShooterOnButton = GpFancyButton(FancyButton.PressType.Toggle) {
        secondary().a
    }.apply {
        setPressed(false)
    }

    fun keepShooterOn(): GpFancyButton = keepShooterOnButton

    fun primary(): Gamepad = gamepad1

    fun secondary(): Gamepad {
        return if (oneController) gamepad1 else gamepad2
    }

    fun update() {
        intakeControl().update()
        intakeEject().update()
        reZeroVision().update()
        moveToParkZone().update()
        stopPark().update()
        flipHive().update()
        deployer().update()
        runTransferFast().update()
        runTransferSlow().update()
        panicShoot().update()
        autoAlignAndStartShoot().update()
        keepShooterOn().update()
    }
}