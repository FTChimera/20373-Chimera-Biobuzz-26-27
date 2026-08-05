package org.firstinspires.ftc.teamcode.opModes;

import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.ivy.commands.Commands;
import com.pedropathing.ivy.groups.Groups;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.library.DriveVector;
import org.firstinspires.ftc.teamcode.library.hardware.gamepad.Button;
import org.firstinspires.ftc.teamcode.library.hardware.gamepad.Bindings;
import org.firstinspires.ftc.teamcode.library.hardware.gamepad.DualGamepad;
import org.firstinspires.ftc.teamcode.library.hardware.webcam.Limelight;
import org.firstinspires.ftc.teamcode.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.MecanumDrive;

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
        DualGamepad gamepads = new DualGamepad(gamepad1, gamepad2);
        Bindings driverBindings = gamepads.gamepad1.bindings(Button.BACK);
        intake = Constants.createIntake(hardwareMap);
        Pose startPose = Constants.RED_DEFAULT_START_POSE;
        drive = Constants.createTeleOpFollower(hardwareMap);
        drive.setStartingPose(startPose);
        drive.setStickCurve((x -> Math.signum(x)*x*x));
        drive.startTeleOpDrive(); // Stop all motors
        limelight = Constants.createLimelight(hardwareMap);

        Scheduler.schedule(
                gamepads.updateCommand(),
                Commands.infinite(() -> limelight.periodic(driverBindings)),
                Commands.infinite(() -> drive.periodic(driverBindings)),
                drive.teleOp(() -> new DriveVector(-gamepads.gamepad1.leftStickY(), gamepads.gamepad1.leftStickX(), gamepads.gamepad1.rightStickX())),
                Groups.loop(intake.setPowerCommand(() -> gamepads.rightTrigger1().getValue() - gamepads.leftTrigger1().getValue())),
                Commands.infinite(() -> {if (gamepads.isPressed1(Button.BACK)) {drive.zeroHeadingOffset();}}).requiring(gamepads, drive)
        );

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        limelight.start(Limelight.Pipeline.APRIL_TAG);
        while (opModeIsActive()) {
            Scheduler.execute();

            // Telemetry
            telemetry.addData("Pose", drive.getPose());
            telemetry.addData("Intake Power", intake.getTargetPower());
            telemetry.update();
        }

    }
}
