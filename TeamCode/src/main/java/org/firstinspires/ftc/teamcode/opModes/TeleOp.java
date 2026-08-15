package org.firstinspires.ftc.teamcode.opModes;

import androidx.annotation.NonNull;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Constants;
import com.rmsftc.library.AllianceColor;
import com.rmsftc.library.hardware.gamepad.GamepadEx;
import com.rmsftc.library.hardware.gamepad.GpBindings;
import com.rmsftc.library.hardware.webcam.Limelight;
import org.firstinspires.ftc.teamcode.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.Mechanisms;
import org.firstinspires.ftc.teamcode.mechanisms.RobotDrive;

@SuppressWarnings({"FieldCanBeLocal", "SpellCheckingInspection"})
public abstract class TeleOp extends LinearOpMode {
    AllianceColor allianceColor;
    Mechanisms mechanisms;
    Intake intake;
    RobotDrive drive;
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
        GamepadEx driver = new GamepadEx(gamepad1);
        GpBindings driverResult = driver.result();
        mechanisms = Constants.createMechanisms(hardwareMap);
        intake = mechanisms.get(Intake.class);
        drive = mechanisms.require(RobotDrive.class);
        drive.setStartingPose(getStartingPose());
        drive.setStickCurve((x -> Math.signum(x)*x*x));
        drive.startTeleOpDrive(); // Stop all motors
        limelight = mechanisms.get(Limelight.class);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        if (limelight != null) {
            limelight.start(Limelight.Pipeline.APRIL_TAG);
        }
        while (opModeIsActive()) {
            driver.update();
            mechanisms.periodic(driverResult);

            // Telemetry
            telemetry.addData("Pose", drive.getPose());
            telemetry.addData("Intake Power", intake == null ? 0 : intake.getTargetPower());
            telemetry.update();
        }

    }

}
