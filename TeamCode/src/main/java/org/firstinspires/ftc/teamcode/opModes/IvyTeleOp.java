package org.firstinspires.ftc.teamcode.opModes;

import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.ivy.commands.Commands;
import com.pedropathing.ivy.groups.Groups;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.library.DriveVector;
import org.firstinspires.ftc.teamcode.library.hardware.webcam.Limelight;
import org.firstinspires.ftc.teamcode.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.MecanumDrive;
import org.firstinspires.ftc.teamcode.mechanisms.Mechanisms;

@TeleOp(name = "Red Ivy TeleOp", group = "0: TeleOp")
@SuppressWarnings({"FieldCanBeLocal", "SpellCheckingInspection"})
public class IvyTeleOp extends LinearOpMode {
    Intake intake;
    MecanumDrive drive;
    Limelight limelight;

    /**
     * Override this method and place your code here.
     * <p>
     * Please do not catch {@link InterruptedException}s that are thrown in your OpMode
     * unless you are doing it to perform some brief cleanup, in which case you must exit
     * immediately afterward. Once the OpMode has been told to stop, your ability to
     * control hardware will be limited.
     */
    @Override
    public void runOpMode() {
        Scheduler.reset();
        setGamepadCurve();
        intake = Mechanisms.intake(hardwareMap);
        Pose startPose = Constants.RED_DEFAULT_START_POSE;
        drive = MecanumDrive.pinpointLocalizer(hardwareMap, startPose);
        drive.startTeleOpDrive(); // Stop all motors
        limelight = Mechanisms.limelight(hardwareMap);

        Scheduler.schedule(
                limelight.updateCommand(),
                drive.periodicCommand(),
                drive.teleOp(() -> new DriveVector(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x)),
                Groups.loop(intake.setPowerCommand(() -> (double) (gamepad1.right_trigger - gamepad1.left_trigger))),
                Commands.infinite(() -> {if (gamepad1.back) {drive.resetHeading(0);}}).requiring(gamepad1, drive)
        );

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        limelight.start(Limelight.Pipeline.APRIL_TAG);
        while (opModeIsActive()) {
            // Update Mechanisms
            limelight.LLUpdate();
            Scheduler.execute();

            // Telemetry
            telemetry.addData("Pose", drive.getPose());
            telemetry.addData("Intake Power", intake.getTargetPower());
            telemetry.update();
        }

    }

    public void setGamepadCurve() {
        drive.setStickCurve(x -> Math.signum(x)*x*x);
    }
}
