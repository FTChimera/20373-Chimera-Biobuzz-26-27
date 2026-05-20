package org.firstinspires.ftc.teamcode.tests;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.library.DriveVector;
import org.firstinspires.ftc.teamcode.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.MecanumDrive;

@TeleOp(name = "Motor Direction Test", group = "1: Tests")
public class MotorDirectionTest extends LinearOpMode {
    /**
     * Use the panels configurable on constants to set directions.
     */
    MecanumDrive drive;
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
        waitForStart();
        do {
            drive = MecanumDrive.pinpointLocalizer(hardwareMap, new Pose(0, 0, 0));
            intake = Constants.createIntake(hardwareMap);
            drive.setTeleOpDrive(new DriveVector(0.3, 0, 0));
            intake.setPower(0.3);
        }
        while (opModeIsActive());
    }
}
