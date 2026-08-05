package org.firstinspires.ftc.teamcode.opModes;

import androidx.annotation.NonNull;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.library.AllianceColor;
import org.firstinspires.ftc.teamcode.library.hardware.gamepad.Button;
import org.firstinspires.ftc.teamcode.library.hardware.gamepad.GamepadEx;
import org.firstinspires.ftc.teamcode.library.hardware.gamepad.GpBindings;
import org.firstinspires.ftc.teamcode.library.hardware.webcam.Limelight;
import org.firstinspires.ftc.teamcode.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.RobotDrive;

@SuppressWarnings({"FieldCanBeLocal", "SpellCheckingInspection"})
public abstract class TeleOp extends LinearOpMode {
    AllianceColor allianceColor;
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
        GpBindings driverResult = driver.result(Button.BACK);
        intake = Constants.createIntake(hardwareMap);
        drive = Constants.createDrive(hardwareMap);
        drive.setStartingPose(getStartingPose());
        drive.setStickCurve((x -> Math.signum(x)*x*x));
        drive.startTeleOpDrive(); // Stop all motors
        limelight = Constants.createLimelight(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        limelight.start(Limelight.Pipeline.APRIL_TAG);
        while (opModeIsActive()) {
            driver.update();
            // Update Mechanisms
            drive.periodic(driverResult);
            limelight.periodic(driverResult);

            // Intake
            intake.setPower(driver.rightTrigger.getValue() - driver.leftTrigger.getValue());

            // Telemetry
            telemetry.addData("Pose", drive.getPose());
            telemetry.addData("Intake Power", intake.getTargetPower());
            telemetry.update();
        }

    }

}
