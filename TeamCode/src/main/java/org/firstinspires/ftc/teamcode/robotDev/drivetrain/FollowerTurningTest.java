package org.firstinspires.ftc.teamcode.robotDev.drivetrain;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Utility;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.pedro.Follower;
import org.firstinspires.ftc.teamcode.planners.CommonKt;
import org.firstinspires.ftc.teamcode.util.AllianceColor;
import org.firstinspires.ftc.teamcode.util.Timer;
import org.firstinspires.ftc.teamcode.util.poses.PoseFactory;

import java.util.concurrent.TimeUnit;

@Utility(name = "Follower Turning Test", description = "Test the follower's turning functionality")
public class FollowerTurningTest extends LinearOpMode {
    Follower follower;
    Timer loopTimer;
    @Override
    public void runOpMode() {
        Constants.PINPOINT_CONFIG.resetMode.set(PinpointLocalizer.ResetMode.RESET_AND_RECALIBRATE_IMU);
        follower = Constants.createFollower(hardwareMap);
        CommonKt.init(AllianceColor.RED);
        follower.localizer.setPose(PoseFactory.deg(0));
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addData("Status", "Initialized.");
        telemetry.addData("Alliance Color", AllianceColor.RED);
        telemetry.addLine("Position your robot facing the blue loading zone from red alliance (heading 0)");


        waitForStart();
        loopTimer = new Timer(TimeUnit.SECONDS);
        do {
            if (isStopRequested()) break;
            double dt = loopTimer.dt();

            if (gamepad1.a) {
                follower.hold(follower.localizer.pose().withHeading(0));
            } else {
                follower.manual(-gamepad1.left_stick_y*0.8, -gamepad1.left_stick_x*0.8, -gamepad1.right_stick_x*0.8,true);
            }

            follower.update();

            telemetry.addLine("====== FOLLOWER TURNING(HOLDING) TEST =======namaste");
            telemetry.addData("Current Pose", follower.localizer.pose());
            telemetry.addData("DelTaTime", dt);
            telemetry.update();
        } while (opModeIsActive());
    }
}
