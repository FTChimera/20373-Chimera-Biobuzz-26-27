package org.firstinspires.ftc.teamcode.opmodes

import com.qualcomm.robotcore.hardware.Gamepad
import org.firstinspires.ftc.teamcode.util.hardware.gamepad.FancyButton
import org.firstinspires.ftc.teamcode.util.hardware.gamepad.GpFancyButton

class Controls(
    var gamepad1: Gamepad,
    var gamepad2: Gamepad,
    val oneController: Boolean
) {

    fun intakeControl(): GpFancyButton {
        return GpFancyButton(FancyButton.PressType.Toggle) {
            secondary().x
        }.apply {
            setPressed(true)
        }
    }

    fun intakeEject(): GpFancyButton =
        GpFancyButton(FancyButton.PressType.Normal) {
            secondary().back
        }

    fun reZeroVision(): GpFancyButton =
        GpFancyButton(FancyButton.PressType.LongPress) {
            secondary().start
        }

    fun moveToParkZone(): GpFancyButton =
        GpFancyButton(FancyButton.PressType.Normal) {
            primary().a
        }

    fun runTransferFast(): GpFancyButton =
        GpFancyButton(FancyButton.PressType.Normal) {
            secondary().right_bumper
        }

    fun runTransferSlow(): GpFancyButton =
        GpFancyButton(FancyButton.PressType.Normal) {
            secondary().left_bumper
        }

    fun panicShoot(): GpFancyButton =
        GpFancyButton(FancyButton.PressType.Normal) {
            secondary().b
        }

    fun autoAlignAndStartShoot(): GpFancyButton =
        GpFancyButton(FancyButton.PressType.Normal) {
            secondary().y
        }

    fun keepShooterOn(): GpFancyButton {
        return GpFancyButton(FancyButton.PressType.Toggle) {
            secondary().y
        }.apply {
            setPressed(false)
        }
    }

    fun primary(): Gamepad = gamepad1

    fun secondary(): Gamepad {
        return if (oneController) gamepad1 else gamepad2
    }
}