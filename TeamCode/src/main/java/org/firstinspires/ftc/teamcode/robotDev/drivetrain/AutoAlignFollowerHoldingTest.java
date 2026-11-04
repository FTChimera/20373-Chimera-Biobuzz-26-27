package org.firstinspires.ftc.teamcode.robotDev.drivetrain;


import static org.firstinspires.ftc.teamcode.util.vars.kt.TypesKt.BoolVar;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Utility;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.pedro.Follower;
import org.firstinspires.ftc.teamcode.planners.CommonKt;
import org.firstinspires.ftc.teamcode.planners.StartPoses;
import org.firstinspires.ftc.teamcode.util.AllianceColor;
import org.firstinspires.ftc.teamcode.util.Timer;
import org.firstinspires.ftc.teamcode.util.game.Field;
import org.firstinspires.ftc.teamcode.util.shotcalc.AutoAimerCalculator;
import org.firstinspires.ftc.teamcode.util.vars.Var;
import org.firstinspires.ftc.teamcode.util.vars.Variables;

import java.util.concurrent.TimeUnit;

@Utility(name = "Auto Align Follower Holding Test", description = "Test the auto align Follower holding functionality")
public class AutoAlignFollowerHoldingTest extends LinearOpMode {
    Follower follower;
    AutoAimerCalculator autoAimerCalculator;
    Timer loopTimer;
    Var<Boolean> holdScale = BoolVar("Hold Scale", false);
    @Override
    public void runOpMode() {
        autoAimerCalculator = new AutoAimerCalculator(AllianceColor.RED);
        Constants.PINPOINT_CONFIG.resetMode.set(PinpointLocalizer.ResetMode.RESET_AND_RECALIBRATE_IMU);
        follower = Constants.createFollower(hardwareMap);
        CommonKt.init(AllianceColor.RED);
        follower.localizer.setPose(StartPoses.gardenArea);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addData("Status", "Initialized.");
        telemetry.addData("Alliance Color", AllianceColor.RED);
        telemetry.addLine("Position your robot on the Garden start pose for auto (RED)");
        holdScale.section.category = "AutoAlignTest";
        holdScale.uploadToFtcDashboard();
        telemetry.update();
        autoAimerCalculator.hiveState = Field.HiveState.UNTIPPED;


        waitForStart();
        loopTimer = new Timer(TimeUnit.SECONDS);
        do {
            if (isStopRequested()) break;
            double dt = loopTimer.dt();
            Pose currentPose = follower.localizer.pose();
            Velocity velocity = follower.localizer.velocity();
            Velocity deltaPose = velocity.scale(dt);

            //deltaPose = Velocity.zero();
            autoAimerCalculator.updatePedro(currentPose, deltaPose);

            if (gamepad1.yWasPressed()) autoAimerCalculator.hiveState = Field.HiveState.UNTIPPED;
            if (gamepad1.xWasPressed()) autoAimerCalculator.hiveState = Field.HiveState.TIPPED;
            if (gamepad1.a) {
                follower.manual(-gamepad1.left_stick_y*0.8, -gamepad1.left_stick_x*0.8, follower.algorithm.calculateHold(
                        follower.drivetrain,
                        autoAimerCalculator.getCorrectedPose().withHeading(autoAimerCalculator.getTargetHeading()),
                        follower.localizer.state(),
                        false, // Use hold scaling
                        dt // in seconds
                ).turn(), true);
            } else {
                follower.manual(-gamepad1.left_stick_y*0.8, -gamepad1.left_stick_x*0.8, -gamepad1.right_stick_x*0.8, true);
            }

            follower.update();

            telemetry.addLine("====== AUTO ALIGN FOLLOWER HOLDING TEST =======namaste");
            telemetry.addData("Current Pose", currentPose);
            telemetry.addData("Compensated Pose", autoAimerCalculator.getCorrectedPose());
            telemetry.addData("DelTaTime", dt);
            telemetry.addData("--- AUTO AIMER CALCULATOR DEBUG STRING", autoAimerCalculator.getDebugString());
            telemetry.addData("Hive State", autoAimerCalculator.hiveState);
            telemetry.update();
        } while (opModeIsActive());
    }
}
