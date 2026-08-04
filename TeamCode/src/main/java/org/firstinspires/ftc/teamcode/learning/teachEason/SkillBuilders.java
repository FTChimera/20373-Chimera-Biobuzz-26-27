package org.firstinspires.ftc.teamcode.learning.teachEason;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.library.hardware.actuators.Direction;
import org.firstinspires.ftc.teamcode.library.hardware.gamepad.Button;
import org.firstinspires.ftc.teamcode.library.hardware.gamepad.GamepadEx;
import org.firstinspires.ftc.teamcode.library.hardware.actuators.Motor;

/**
 * A simple tank drive OpMode to demonstrate the use of the Motor class and basic control flow in an OpMode.
 * Added intakeMotor to demonstrate the use of multiple motors and direction control.
 */
@TeleOp(name = "Skill Builders", group = "Teach Eason")
public class SkillBuilders extends LinearOpMode {
    Motor leftDrive, rightDrive;
    Motor intakeMotor;

    @Override
    public void runOpMode() {
        leftDrive = new Motor(hardwareMap, "left_drive");
        rightDrive = new Motor(hardwareMap, "right_drive");
        intakeMotor = new Motor(hardwareMap, "intake_motor");

        leftDrive.setDirection(Direction.FORWARD);
        rightDrive.setDirection(Direction.REVERSE);
        intakeMotor.setDirection(Direction.FORWARD);

        waitForStart();
        GamepadEx driver = new GamepadEx(gamepad1);
        while (opModeIsActive()) {
            driver.update();
            double forward = driver.leftStickY();
            double rotate = driver.rightStickX();

            double leftPower = forward + rotate;
            double rightPower = forward - rotate;
            double intakePower = 0;
            if (driver.isPressed(Button.Y)) {
                intakePower = 1;
            } else if (driver.isPressed(Button.A)) {
                intakePower = -0.5;
            }

            leftDrive.setPower(leftPower);
            rightDrive.setPower(rightPower);
            intakeMotor.setPower(intakePower);
        }
    }
}
