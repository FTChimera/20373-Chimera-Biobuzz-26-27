package org.firstinspires.ftc.teamcode.tests;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.library.DriveVector;
import org.firstinspires.ftc.teamcode.library.hardware.actuators.Direction;
import org.firstinspires.ftc.teamcode.mechanisms.Drivetrain;
import org.firstinspires.ftc.teamcode.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.MecanumDrive;

import java.util.Arrays;
import java.util.Locale;

@TeleOp(name = "Motor Direction Test", group = "1: Tests")
public class MotorDirectionTest extends LinearOpMode {
    /**
     * Use the panels configurable on constants to set directions.
     */
    Drivetrain drive;
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
        drive = Constants.createDrivetrain(hardwareMap);
        drive.startTeleOpDrive();
        intake = Constants.createIntake(hardwareMap);
        waitForStart();
        if (gamepad1.dpadDownWasPressed()) {
            reverseDriveMotor(0);
        }
        if (gamepad1.dpadUpWasPressed()) {
            reverseDriveMotor(1);
        }
        if (gamepad1.dpadLeftWasPressed()) {
            reverseDriveMotor(2);
        }
        if (gamepad1.dpadRightWasPressed()) {
            reverseDriveMotor(3);
        }
        if (gamepad1.aWasPressed()) {
            reverseIntake();
        }
        drive.setTeleOpDrive(new DriveVector(0.3, 0, 0), 0);
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
