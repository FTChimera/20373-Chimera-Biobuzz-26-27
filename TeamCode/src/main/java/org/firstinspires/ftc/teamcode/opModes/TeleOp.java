package org.firstinspires.ftc.teamcode.opModes;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Blackboard;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.library.AllianceColor;
import org.firstinspires.ftc.teamcode.library.DriveVector;
import org.firstinspires.ftc.teamcode.library.hardware.gamepad.Button;
import org.firstinspires.ftc.teamcode.library.hardware.gamepad.GamepadEx;
import org.firstinspires.ftc.teamcode.library.hardware.webcam.Limelight;
import org.firstinspires.ftc.teamcode.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.MecanumDrive;
import org.jetbrains.annotations.Async;

@SuppressWarnings({"FieldCanBeLocal", "SpellCheckingInspection"})
public class TeleOp extends LinearOpMode {
    AllianceColor allianceColor;
    Intake intake;
    MecanumDrive drive;
    Limelight limelight;
    GamepadEx gpad1, gpad2;

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
    @Override
    public void runOpMode() {
        gpad1 = new GamepadEx(gamepad1);
        gpad2 = new GamepadEx(gamepad2);
        initializeGamepadSettings();
        intake = new Intake(hardwareMap);
        Pose startPose = Blackboard.pose;
        if (startPose == null) {
            startPose = allianceColor == AllianceColor.RED ? Constants.RED_DEFAULT_START_POSE : Constants.BLUE_DEFAULT_START_POSE;
        }
        drive = new MecanumDrive(hardwareMap, startPose, true);
        drive.startTeleopDrive(); // Stop all motors
        limelight = new Limelight(hardwareMap, Constants.LIMELIGHT_CAMERA_NAME);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        limelight.start(Limelight.Pipeline.APRIL_TAG);
        while (opModeIsActive()) {
            // Update Mechanisms
            gpad1.update();
            gpad2.update();
            drive.periodic();
            limelight.LLUpdate();

            // Imu reset yaw
            if (gpad1.isPressed(Button.BACK)) {
                drive.resetHeading(0);
            }

            // Drive
            double forward = gpad1.leftStickY(); // Don't need to negate because it is already negated
            double strafe = gpad1.leftStickX();
            double turn = gpad1.rightStickX();
            drive.setTeleOpDrive(new DriveVector(forward, strafe, turn));

            // Intake
            intake.setPower(gpad1.rightTrigger.getValue() - gpad1.leftTrigger.getValue());

            // Telemetry
            telemetry.addData("Pose", drive.getPose());
            telemetry.addData("Intake Power", intake.getTargetPower());
            telemetry.update();
        }

    }

    public void initializeGamepadSettings() {
        gpad1.setStickCurve(x -> Math.signum(x)*x*x);
        gpad2.setStickCurve(x -> Math.signum(x)*x*x);
    }
}
