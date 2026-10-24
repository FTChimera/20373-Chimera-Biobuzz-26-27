package org.firstinspires.ftc.teamcode.robotDev.drivetrain;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.ValueProvider;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.config.ConfigVar;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.pedropathing.utils.Angle;
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

import java.util.concurrent.TimeUnit;

@Utility(name = "Auto Align Test", description = "Test the auto align heading functionality")
public class AutoAlignTest extends LinearOpMode {
    private ConfigVar<Double> Kp_Turn = ConfigVar.of(0.025);
    ValueProvider<Double> kpTurnProvider = new ValueProvider<Double>() {
        @Override
        public Double get() {
            return Kp_Turn.get();
        }

        @Override
        public void set(Double value) {
            Kp_Turn.set(value);
        }
    };
    Follower follower;
    AutoAimerCalculator autoAimerCalculator;
    Timer loopTimer;
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
        FtcDashboard.getInstance().addConfigVariable("Auto Align Test", "Kp Turn", kpTurnProvider);
        autoAimerCalculator.hiveState = Field.HiveState.UNTIPPED;


        waitForStart();
        loopTimer = new Timer(TimeUnit.SECONDS);
        do {
            if (isStopRequested()) break;
            double dt = loopTimer.dt();
            Pose currentPose = follower.localizer.pose();
            Velocity velocity = follower.localizer.velocity();
            Velocity deltaPose = velocity.scale(dt);

            deltaPose = Velocity.zero(); //TODO Comment this out
            autoAimerCalculator.updatePedro(currentPose, deltaPose);

            if (gamepad1.a) {
                follower.manual(-gamepad1.left_stick_y*0.8, gamepad1.left_stick_x*0.8, Angle.error(currentPose.heading(), autoAimerCalculator.getCorrectedPose().heading()) * Kp_Turn.get());
            } else {
                follower.manual(-gamepad1.left_stick_y*0.8, gamepad1.left_stick_x*0.8, gamepad1.right_stick_x*0.8);
            }

            follower.update();

            telemetry.addLine("====== AUTO ALIGN TEST =======namaste");
            telemetry.addData("Current Pose", currentPose);
            telemetry.addData("Compensated Pose", autoAimerCalculator.getCorrectedPose());
            telemetry.addData("DelTaTime", dt);
            telemetry.addData("--- AUTO AIMER CALCULATOR DEBUG STRING", autoAimerCalculator.getDebugString());
            telemetry.update();
        } while (opModeIsActive());
    }
}
