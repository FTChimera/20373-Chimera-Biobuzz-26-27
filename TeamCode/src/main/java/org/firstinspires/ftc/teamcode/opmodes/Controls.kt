package org.firstinspires.ftc.teamcode.opmodes

import com.qualcomm.robotcore.hardware.Gamepad
import org.firstinspires.ftc.teamcode.util.hardware.gamepad.FancyButton
import org.firstinspires.ftc.teamcode.util.hardware.gamepad.GpFancyButton

class Controls(
    var gamepad1: Gamepad,
    var gamepad2: Gamepad,
    val oneController: Boolean
) {

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
        primary().a
    }

    fun moveToParkZone(): GpFancyButton = moveToParkZoneButton

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
        secondary().y
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
        runTransferFast().update()
        runTransferSlow().update()
        panicShoot().update()
        autoAlignAndStartShoot().update()
        keepShooterOn().update()
    }
}