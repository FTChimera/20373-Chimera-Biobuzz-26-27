package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Utility;

import org.firstinspires.ftc.teamcode.Constants;
import com.rmsftc.library.DriveVector;
import com.rmsftc.library.hardware.actuators.Direction;
import com.rmsftc.library.hardware.gamepad.Button;
import com.rmsftc.library.hardware.gamepad.GamepadEx;
import org.firstinspires.ftc.teamcode.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.RobotDrive;

import java.util.Arrays;

@Utility(name = "Motor Direction Test")
public class MotorDirectionTest extends LinearOpMode {
    /**
     * Use the panels configurable on constants to set directions.
     */
    RobotDrive drive;
    Intake intake;
    /**
     * Override this method and place your code here.
     * <p>
     * Please do not catch {@link InterruptedException}s that are thrown in your OpMode
     * unless you are doing it to perform some brief cleanup, in which case you must exit
     * immediately afterward. Once the OpMode has been told to stop, your ability to
     * control hardware will be limited.
     *
     * @throws InterruptedException When the OpMode is stopped while calling a method
     *                              that can throw {@link InterruptedException}
     */
    @Override
    public void runOpMode() {
        drive = Constants.createDrive(hardwareMap);
        drive.startTeleOpDrive();
        intake = Constants.createIntake(hardwareMap);
        GamepadEx driver = new GamepadEx(gamepad1);
        waitForStart();
        while (opModeIsActive()) {
            driver.update();
            if (driver.wasJustPressed(Button.DPAD_DOWN)) {
                reverseDriveMotor(0);
            }
            if (driver.wasJustPressed(Button.DPAD_UP)) {
                reverseDriveMotor(1);
            }
            if (driver.wasJustPressed(Button.DPAD_LEFT)) {
                reverseDriveMotor(2);
            }
            if (driver.wasJustPressed(Button.DPAD_RIGHT)) {
                reverseDriveMotor(3);
            }
            if (driver.wasJustPressed(Button.A)) {
                reverseIntake();
            }
            drive.setTeleOpDrive(new DriveVector(0.3, 0, 0), false);
            intake.setPower(0.3);
            telemetry.addLine("-".repeat(30));
            telemetry.addLine("INSTRUCTIONS");
            telemetry.addLine("-".repeat(30));
            telemetry.addData("Drive Motors", "Check if the robot is moving forward");
            telemetry.addData("0:Front Left Motor", "Dpad Down");
            telemetry.addData("1:Front Right Motor", "Dpad Up");
            telemetry.addData("2:Back Left Motor", "Dpad Left");
            telemetry.addData("3:Back Right Motor", "Dpad Right");
            telemetry.addData("4:Intake", "A");
            telemetry.addLine("-".repeat(30));
            telemetry.addData("Motor Directions", getMotorDirectionsString());
            telemetry.update();
        }
    }

    void reverseDriveMotor(int index) {
        Arrays.asList(drive.getDriveMotors()).get(index).reverse();
    }
    void reverseIntake() {
        intake.Intake_Motor.reverse();
    }

    String getMotorDirectionsString() {
        return getDirString(0) + " " + getDirString(1) + " " + getDirString(2) + " " + getDirString(3) + " | " + getDirString(4);
    }

    String getDirString(int idx) {
        if (idx == 4) return intake.Intake_Motor.getDirection() == Direction.FORWARD? "F" : "R";
        return drive.getDriveMotors()[idx].getDirection() == Direction.FORWARD? "F" : "R";
    }
}
