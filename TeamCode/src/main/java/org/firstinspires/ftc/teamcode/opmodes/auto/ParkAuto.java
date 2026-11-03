
package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.pedropathing.api.Paths;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.mechanisms.Robot;
import org.firstinspires.ftc.teamcode.opmodes.MainTeleOp;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.pedro.Follower;
import org.firstinspires.ftc.teamcode.util.AllianceColor;
import org.firstinspires.ftc.teamcode.util.RobotMode;
import org.firstinspires.ftc.teamcode.util.poses.Pose2d;
import org.firstinspires.ftc.teamcode.util.poses.PoseFactory;

@Autonomous(name = "Park Auto", preselectTeleOp = MainTeleOp.NAME)
public class ParkAuto extends LinearOpMode {

    enum StartPose {
        RED_UNTIPPED(60, 10, 0),
        RED_TIPPED(32, 134, 0),
        BLUE_UNTIPPED(84, 130, 180),
        BLUE_TIPPED(112, 10, 180);

        public final Pose2d pose;

        StartPose(double x, double y, double heading) {
            pose = PoseFactory.deg(x, y, heading);
        }
    }

    private StartPose selectedPose = StartPose.RED_TIPPED;

    @Override
    public void runOpMode() {

        // Selection happens before Robot initialization.
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

        AllianceColor alliance =
                selectedPose.name().startsWith("RED")
                        ? AllianceColor.RED
                        : AllianceColor.BLUE;

        Follower follower = Constants.createFollower(hardwareMap).withLogger(followerLog -> telemetry.addLine(followerLog.toString()));
        Pose2d PARK_POSE = alliance.isRed()? PoseFactory.deg(8,104,0) : PoseFactory.deg(135, 38, 180);
        follower.localizer.setPose(selectedPose.pose);
        if (isStopRequested()) return;
        follower.follow(Paths.line(selectedPose.pose, PARK_POSE).linear(selectedPose.pose, PARK_POSE));
        try {
            while (opModeIsActive() && follower.following()) {
                follower.update();
                telemetry.update();
            }
        } finally {
            follower.stop();
            follower.drivetrain.stop();
        }
        requestOpModeStop();
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
