package org.firstinspires.ftc.teamcode.opModes;

import androidx.annotation.NonNull;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.library.AllianceColor;
import org.firstinspires.ftc.teamcode.library.game.RobotMode;
import org.firstinspires.ftc.teamcode.library.game.State;
import org.firstinspires.ftc.teamcode.library.hardware.gamepad.GamepadEx;
import org.firstinspires.ftc.teamcode.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.Robot;
import org.firstinspires.ftc.teamcode.mechanisms.RobotDrive;
import org.firstinspires.ftc.teamcode.mechanisms.Vision;

@SuppressWarnings({"FieldCanBeLocal", "SpellCheckingInspection"})
public abstract class TeleOp extends LinearOpMode {
    AllianceColor allianceColor;
    Robot robot;
    Intake intake;
    RobotDrive drive;
    Vision vision;

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
        State robotState = new State(RobotMode.TELEOP);
        robot = Constants.createRobot(hardwareMap, RobotMode.TELEOP);
        intake = robot.getIntake();
        drive = robot.requireDrive();
        robot.setStartingPose(getStartingPose());
        robot.setDriveStickCurve(x -> Math.signum(x) * x * x);
        robot.startTeleOpDrive();
        vision = robot.requireVision();
        robotState.setGpBindings(driver.result());
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        robot.start();
        robot.startVision(Vision.Pipeline.APRIL_TAG);
        while (opModeIsActive()) {
            driver.update();
            robot.periodic(robotState);

            // Telemetry
            telemetry.addData("Pose", drive.getPose());
            telemetry.addData("Intake Power", intake == null ? 0 : intake.getTargetPower());
            telemetry.update();
        }
        robot.stopVision();

    }

}
