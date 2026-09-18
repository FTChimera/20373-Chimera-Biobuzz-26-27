package org.firstinspires.ftc.teamcode.tele;

import androidx.annotation.NonNull;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.configs.Blackboard;
import org.firstinspires.ftc.teamcode.library.AllianceColor;
import org.firstinspires.ftc.teamcode.library.game.RobotMode;
import org.firstinspires.ftc.teamcode.library.game.State;
import org.firstinspires.ftc.teamcode.library.hardware.gamepad.DualGamepad;
import org.firstinspires.ftc.teamcode.mechanisms.Robot;
import org.firstinspires.ftc.teamcode.mechanisms.Vision;

@SuppressWarnings({"FieldCanBeLocal", "SpellCheckingInspection"})
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = Constants.TELEOP_NAME, group = Constants.COMPETITION_GROUP)
public class TeleOp extends LinearOpMode {
    AllianceColor allianceColor;
    Robot robot;

    /**
     * Override this method and place your code here.
     * <p>
     * Please do not catch {@link InterruptedException}s that are thrown in your OpMode
     * unless you are doing it to perform some brief cleanup, in which case you must exit
     * immediately afterward. Once the OpMode has been told to stop, your ability to
     * control hardware will be limited.
     */
    @NonNull
    public Pose getStartingPoseAndSetAlliance() {
        if (allianceColor == null && Blackboard.get(Blackboard.Caption.ALLIANCE) != null) {
            allianceColor = Blackboard.get(Blackboard.Caption.ALLIANCE).getValue();
        }
        if (Blackboard.get(Blackboard.Caption.POSE) != null && Blackboard.get(Blackboard.Caption.POSE).getValue() != null) {
            return Blackboard.get(Blackboard.Caption.POSE).getValue();
        }
        if (allianceColor != null) {
            return allianceColor.isRed()? Constants.RED_DEFAULT_START_POSE : Constants.BLUE_DEFAULT_START_POSE;
        } else {
            allianceColor = AllianceColor.RED;
        }
        return Constants.RED_DEFAULT_START_POSE;
    }
    @Override
    public void runOpMode() {
        DualGamepad drivers = new DualGamepad(gamepad1, gamepad2);
        State robotState = new State(RobotMode.TELEOP);
        robot = Constants.createRobot(hardwareMap, RobotMode.TELEOP);
        robot.setStartingPose(getStartingPoseAndSetAlliance());
        robot.setAllianceColor(allianceColor);
        robot.setDriveStickCurve(x -> Math.signum(x) * x*1.1);
        robot.startTeleOpDrive();
        robotState.setGpBindings(drivers.result());
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        robot.start();
        robot.startVision(Vision.Pipeline.APRIL_TAG);
        while (opModeIsActive()) {
            drivers.update();
            robot.periodic(robotState);

            // Telemetry
            telemetry.addData("Pose", robot.getPose());
            telemetry.addData("Intake Power", robot.getIntakePower());
            telemetry.update();
        }
        robot.stop();

    }
    protected void setAllianceColor(AllianceColor color) {
        Blackboard.put(new Blackboard.Item(Blackboard.Caption.ALLIANCE, color));
    }

    //These are optional; use the main one instead. This one is for if alliance is broken.
    @com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Red TeleOp") public static class Red_Tele extends TeleOp {@Override public void runOpMode() {setAllianceColor(AllianceColor.RED); super.runOpMode();}}
    @com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Blue TeleOp") public static class Blue_Tele extends TeleOp {@Override public void runOpMode() {setAllianceColor(AllianceColor.BLUE); super.runOpMode();}}
}
