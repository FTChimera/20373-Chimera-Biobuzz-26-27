package org.firstinspires.ftc.teamcode.opModes;

import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.ivy.commands.Commands;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.library.game.State;
import org.firstinspires.ftc.teamcode.library.hardware.gamepad.DualGamepad;
import org.firstinspires.ftc.teamcode.library.hardware.gamepad.GpBindings;
import org.firstinspires.ftc.teamcode.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.Robot;
import org.firstinspires.ftc.teamcode.mechanisms.RobotDrive;
import org.firstinspires.ftc.teamcode.mechanisms.Vision;

@TeleOp(name = "Red Ivy TeleOp", group = "0: TeleOp")
@SuppressWarnings({"FieldCanBeLocal", "SpellCheckingInspection"})
public class IvyTeleOp extends LinearOpMode {
    Robot robot;
    Intake intake;
    RobotDrive drive;
    Vision vision;

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
        GpBindings driverResult = gamepads.getGamepad1().result();
        robot = Constants.createRobot(hardwareMap);
        intake = robot.get(Intake.class);
        Pose startPose = Constants.RED_DEFAULT_START_POSE;
        drive = robot.require(RobotDrive.class);
        drive.setStartingPose(startPose);
        drive.setStickCurve((x -> Math.signum(x)*x*x));
        drive.startTeleOpDrive(); // Stop all motors
        vision = robot.get(Vision.class);
        State robotState = new State();
        robotState.setGpBindings(driverResult);
        Scheduler.schedule(
                gamepads.updateCommand(),
                Commands.infinite(() -> robot.periodic(robotState))
        );

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        if (vision != null) vision.start(Vision.Pipeline.APRIL_TAG);
        while (opModeIsActive()) {
            Scheduler.execute();

            // Telemetry
            telemetry.addData("Pose", drive.getPose());
            telemetry.addData("Intake Power", intake == null ? 0 : intake.getTargetPower());
            telemetry.update();
        }
        vision.stop();

    }
}
