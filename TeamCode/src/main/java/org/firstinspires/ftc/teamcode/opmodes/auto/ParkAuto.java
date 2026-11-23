package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.pedropathing.api.Paths;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.rmsftc.ftc.ArchitectureManager;
import com.rmsftc.ftc.ArchitecturalOpMode;

import org.firstinspires.ftc.teamcode.opmodes.MainTeleOp;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.pedro.Follower;
import org.firstinspires.ftc.teamcode.planners.ParkAutoStartPoses;
import org.firstinspires.ftc.teamcode.planners.commands.DrivetrainCommands;
import org.firstinspires.ftc.teamcode.util.AllianceColor;
import org.firstinspires.ftc.teamcode.util.poses.Pose2d;

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
        while (opModeInInit()) {
            handleSelection();

            telemetry.addLine("=== PARK AUTO CONFIGURATION ===");
            telemetry.addData("Selected Start", selectedPose);
            telemetry.addData("X", selectedPose.pose.getX());
            telemetry.addData("Y", selectedPose.pose.getY());
            telemetry.addLine("");
            telemetry.addLine("D-PAD LEFT / RIGHT: Change start pose");
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
                Paths.line(selectedPose.pose, parkPose).linear(selectedPose.pose, parkPose)));
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

    private void handleSelection() {
        if (gamepad1.dpadRightWasPressed()) {
            StartPose[] poses = StartPose.values();
            selectedPose = poses[
                    (selectedPose.ordinal() + 1) % poses.length
                    ];
        }

        if (gamepad1.dpadLeftWasPressed()) {
            StartPose[] poses = StartPose.values();
            selectedPose = poses[
                    (selectedPose.ordinal() - 1 + poses.length)
                            % poses.length
                    ];
        }
    }
}
