package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.pedropathing.api.Paths;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.rmsftc.ftc.ArchitectureManager;
import com.rmsftc.ftc.ArchitecturalOpMode;
import com.rmsftc.utils.ftc.telemetry.GamepadMenu;
import com.rmsftc.utils.ftc.telemetry.TelemetryLogger;
import com.rmsftc.utils.telemetry.selection.AnswerChoice;
import com.rmsftc.utils.telemetry.selection.Choices;
import com.rmsftc.utils.telemetry.selection.Prompter;
import com.rmsftc.utils.telemetry.selection.Question;
import com.rmsftc.utils.telemetry.selection.Select;

import org.firstinspires.ftc.teamcode.opmodes.MainTeleOp;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.pedro.Follower;
import org.firstinspires.ftc.teamcode.planners.ParkAutoStartPoses;
import org.firstinspires.ftc.teamcode.planners.commands.DrivetrainCommands;
import com.rmsftc.utils.AllianceColor;
import org.firstinspires.ftc.teamcode.util.poses.Pose2d;
import org.firstinspires.ftc.teamcode.util.game.Field;

@Autonomous(name = "Park Auto", preselectTeleOp = MainTeleOp.NAME)
public class ParkAuto extends ArchitecturalOpMode {

    enum StartPose {
        RED_UNTIPPED(ParkAutoStartPoses.redUntippedStart()),
        RED_TIPPED(ParkAutoStartPoses.redTippedStart()),
        BLUE_UNTIPPED(ParkAutoStartPoses.blueUntippedStart()),
        BLUE_TIPPED(ParkAutoStartPoses.blueTippedStart());

        public final Pose2d pose;

        StartPose(Pose2d pose) {
            this.pose = pose;
        }
    }

    private StartPose selectedPose = StartPose.RED_TIPPED;
    private Follower follower;

    @Override
    protected void onInit() {
        // Selection happens before Robot initialization. onInit() runs
        // during the init phase, so the selection loop blocks here
        // exactly as it did in runOpMode().
        Prompter prompter = new Prompter(new TelemetryLogger(telemetry),
                new Question("Start pose", new Choices(
                        selectedPose.ordinal(),
                        Select.a(StartPose.RED_UNTIPPED.name()),
                        Select.a(StartPose.RED_TIPPED.name()),
                        Select.a(StartPose.BLUE_UNTIPPED.name()),
                        Select.a(StartPose.BLUE_TIPPED.name()))));
        GamepadMenu menu = new GamepadMenu(prompter, gamepad1);

        while (opModeInInit()) {
            menu.update();
            // PLAY confirms whatever is highlighted (or already answered).
            AnswerChoice pick = menu.isFinished()
                    ? menu.getAnswers().get(0)
                    : prompter.current().choices.getSelected();
            selectedPose = StartPose.valueOf(pick.name);

            telemetry.addLine("=== PARK AUTO CONFIGURATION ===");
            telemetry.addData("Selected Start", selectedPose);
            telemetry.addData("X", selectedPose.pose.getX());
            telemetry.addData("Y", selectedPose.pose.getY());
            telemetry.addLine("");
            telemetry.addLine("D-PAD UP / DOWN: Change start pose, A: Confirm");
            telemetry.addLine("Press PLAY to confirm");

            telemetry.update();
        }
    }

    @Override
    protected void build(ArchitectureManager manager) {
        AllianceColor alliance =
                selectedPose.name().startsWith("RED")
                        ? AllianceColor.RED
                        : AllianceColor.BLUE;

        follower = Constants.createFollower(hardwareMap).withLogger(followerLog -> telemetry.addLine(followerLog.toString()));
        Pose2d parkPose = ParkAutoStartPoses.destination(alliance);
        follower.localizer.setPose(selectedPose.pose);
        if (isStopRequested()) return;
        manager.schedule(DrivetrainCommands.follow(follower,
                new Field(alliance, 17, 17).generateTransitPath(selectedPose.pose, parkPose)));
    }

    @Override
    protected void onLoop() {
        follower.update();
        if (!follower.following()) {
            interrupt();
        }
    }

    @Override
    public void onStop() {
        if (follower != null) {
            follower.stop();
            follower.drivetrain.stop();
        }
        super.onStop();
    }
}
