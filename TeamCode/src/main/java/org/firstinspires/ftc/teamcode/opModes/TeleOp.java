package org.firstinspires.ftc.teamcode.opModes;

import androidx.annotation.NonNull;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Blackboard;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.library.AllianceColor;
import org.firstinspires.ftc.teamcode.library.DriveVector;
import org.firstinspires.ftc.teamcode.library.hardware.webcam.Limelight;
import org.firstinspires.ftc.teamcode.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.MecanumDrive;
import org.firstinspires.ftc.teamcode.mechanisms.Mechanisms;

@SuppressWarnings({"FieldCanBeLocal", "SpellCheckingInspection"})
public abstract class TeleOp extends LinearOpMode {
    AllianceColor allianceColor;
    Intake intake;
    MecanumDrive drive;
    Limelight limelight;

    // Constructor for TeleOp, takes in the alliance color.
    public TeleOp(AllianceColor col) {
      this.allianceColor = col;
    }

    /**
     * Override this method and place your code here.
     * <p>
     * Please do not catch {@link InterruptedException}s that are thrown in your OpMode
     * unless you are doing it to perform some brief cleanup, in which case you must exit
     * immediately afterward. Once the OpMode has been told to stop, your ability to
     * control hardware will be limited.
     */
    @NonNull
    protected abstract Pose getStartingPose();
    @Override
    public void runOpMode() {
        initializeGamepadSettings();
        intake = Mechanisms.intake(hardwareMap);
        drive = MecanumDrive.pinpointLocalizer(hardwareMap, getStartingPose());
        drive.startTeleopDrive(); // Stop all motors
        limelight = Mechanisms.limelight(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        limelight.start(Limelight.Pipeline.APRIL_TAG);
        while (opModeIsActive()) {
            // Update Mechanisms
            drive.periodic();
            limelight.LLUpdate();

            // Imu reset yaw
            if (gamepad1.back) {
                drive.resetHeading(0);
            }

            // Drive
            double forward = gamepad1.left_stick_y; // Don't need to negate because it will be in drive vector
            double strafe = gamepad1.left_stick_x;
            double turn = gamepad1.right_stick_x;
            drive.setTeleOpDrive(new DriveVector(forward, strafe, turn));

            // Intake
            intake.setPower(gamepad1.right_trigger - gamepad1.left_trigger);

            // Telemetry
            telemetry.addData("Pose", drive.getPose());
            telemetry.addData("Intake Power", intake.getTargetPower());
            telemetry.update();
        }

    }

    public void initializeGamepadSettings() {
        drive.setStickCurve(x -> Math.signum(x)*x*x);
    }
}
