package org.firstinspires.ftc.teamcode.opModes;

import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.ivy.commands.Commands;
import com.pedropathing.ivy.groups.Groups;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.library.DriveVector;
import org.firstinspires.ftc.teamcode.library.hardware.gamepad.Button;
import org.firstinspires.ftc.teamcode.library.hardware.gamepad.GamepadEx;
import org.firstinspires.ftc.teamcode.library.hardware.webcam.Limelight;
import org.firstinspires.ftc.teamcode.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.MecanumDrive;

@TeleOp(name = "Red Ivy TeleOp", group = "0: TeleOp")
@SuppressWarnings({"FieldCanBeLocal", "SpellCheckingInspection"})
public class IvyTeleOp extends LinearOpMode {
    Intake intake;
    MecanumDrive drive;
    Limelight limelight;
    GamepadEx gpad1, gpad2;

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
        gpad1 = new GamepadEx(gamepad1);
        gpad2 = new GamepadEx(gamepad2);
        setGamepadCurve();
        intake = new Intake(hardwareMap);
        Pose startPose = Constants.RED_DEFAULT_START_POSE;
        drive = new MecanumDrive(hardwareMap, startPose, true);
        drive.startTeleopDrive(); // Stop all motors
        limelight = new Limelight(hardwareMap, Constants.LIMELIGHT_CAMERA_NAME);

        Scheduler.schedule(
                drive.periodicCommand(),
                drive.teleOp(() -> new DriveVector(gpad1.leftStickY(), gpad1.leftStickX(), gpad1.rightStickX())),
                Groups.loop(intake.setPowerCommand(() -> gpad1.rightTrigger.getValue() - gpad1.leftTrigger.getValue())),
                Commands.infinite(() -> {if (gpad1.isPressed(Button.BACK)) {drive.resetHeading(0);}}).requiring(gpad1, drive)
        );

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        limelight.start(Limelight.Pipeline.APRIL_TAG);
        while (opModeIsActive()) {
            // Update Mechanisms
            gpad1.update();
            gpad2.update();
            limelight.LLUpdate();
            Scheduler.execute();

            // Telemetry
            telemetry.addData("Pose", drive.getPose());
            telemetry.addData("Intake Power", intake.getTargetPower());
            telemetry.update();
        }

    }

    public void setGamepadCurve() {
        gpad1.setStickCurve(x -> Math.signum(x)*x*x);
        gpad2.setStickCurve(x -> Math.signum(x)*x*x);
    }
}
