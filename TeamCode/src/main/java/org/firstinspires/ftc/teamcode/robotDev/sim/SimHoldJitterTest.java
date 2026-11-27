package org.firstinspires.ftc.teamcode.robotDev.sim;

import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.rmsftc.utils.AllianceColor;
import com.rmsftc.utils.DriveVector;
import com.rmsftc.utils.RobotMode;
import com.rmsftc.utils.math.Pose;
import org.firstinspires.ftc.teamcode.mechanisms.Robot;
import org.firstinspires.ftc.teamcode.mechanisms.Shooter;
import org.firstinspires.ftc.teamcode.planners.CommonKt;
import org.firstinspires.ftc.teamcode.planners.Constants;

@TeleOp(name = "Sim #4 Stationary Y Aim Jitter", group = "Sim diagnostics")
public class SimHoldJitterTest extends OpMode {
    private static final double ACQUIRE_TIMEOUT_S = 8, ACQUIRE_DWELL_S = 0.5, HOLD_S = 5;
    private static final double DRIFT_LIMIT_IN = 0.5, SPEED_LIMIT_IPS = 0.5, ANGULAR_REST_RAD_S = 0.1;
    private static final double MAX_SAMPLE_GAP_S = 0.1;
    private Robot robot;
    private boolean savedSotm, savedBrake, captured, done, holding;
    private PinpointLocalizer.ResetMode savedReset;
    private final ShooterAimMetrics.Dwell acquisition = new ShooterAimMetrics.Dwell();
    private final ShooterAimMetrics.Heading samples = new ShooterAimMetrics.Heading();
    private long acquireStart, holdStart;
    private Pose holdPose;
    private double elapsed, acquisitionTime = Double.NaN, target, actual, speed, drift, maxDrift, maxSpeed;
    private double mainTarget, counterTarget, main, counter;
    private double lastSampleTime, maxSampleGap;
    private boolean wheelGate, headingGate;
    private String result = "INIT: place robot at RED garden shoot pose; empty shooter";

    @Override public void init() {
        boolean success = false;
        try {
            savedSotm = Shooter.useSotm.get();
            savedBrake = org.firstinspires.ftc.teamcode.pedro.Constants.FOLLOWER_BRAKE_AT_END.get();
            savedReset = org.firstinspires.ftc.teamcode.pedro.Constants.PINPOINT_CONFIG.resetMode.get();
            captured = true;
            CommonKt.init(AllianceColor.RED);
            Pose startPose = new Pose(Constants.ShootPoses.garden.x(), Constants.ShootPoses.garden.y(),
                    Constants.ShootPoses.garden.heading());
            robot = new Robot(telemetry, hardwareMap, AllianceColor.RED, RobotMode.TELEOP, startPose);
            robot.setSmartIntakeControl(false);
            robot.setSmartShooterControl(false);
            robot.debug = false;
            Shooter.useSotm.set(true);
            display();
            success = true;
        } finally {
            if (!success) finish("FAIL: initialization did not complete");
        }
    }

    @Override public void start() {
        boolean success = false;
        try {
            robot.start();
            acquireStart = System.nanoTime();
            result = "ACQUIRE (excluded from jitter)";
            success = true;
        } finally {
            if (!success) finish("FAIL: start did not complete");
        }
    }

    @Override public void loop() {
        if (done) { display(); return; }
        boolean success = false;
        try {
            Shooter.useSotm.set(true);
            // MainTeleOp driveLoop's neutral gp2-Y behavior, recomputed each tick.
            robot.setTeleOpDrive(new DriveVector(0, 0, 0), true, false);
            robot.setAutoAlign();
            robot.setSmartShooterControl(true);
            robot.transfer.stop(); // Aim-only: show gates, deliberately never feed.
            robot.update(); // Sole robot/localization tick.
            target = robot.autoAimerCalculator.getTargetHeading();
            actual = robot.getHeading();
            speed = Math.hypot(robot.getVelocity().vx, robot.getVelocity().vy);
            mainTarget = robot.shooter.mainMotor.targetTicksPerSecond;
            counterTarget = robot.shooter.counterRoller.targetTicksPerSecond;
            main = robot.shooter.mainMotor.getTicksPerSecond();
            counter = robot.shooter.counterRoller.getTicksPerSecond();
            headingGate = ShooterAimMetrics.headingReady(target, actual, Constants.TeleOp.AutoFire.HEADING_TOLERANCE_DEG_DEFAULT);
            wheelGate = robot.shooter.upToSpeed();
            long now = System.nanoTime();
            elapsed = (now - (holding ? holdStart : acquireStart)) / 1e9;
            Pose pose = robot.getPose();
            if (!ShooterAimMetrics.finite(target, actual, speed, mainTarget, counterTarget, main, counter,
                    pose.x(), pose.y(), robot.getVelocity().omega, elapsed)) {
                finish("FAIL: nonfinite measurement");
            } else if (!holding) {
                boolean acquired = headingGate && speed <= SPEED_LIMIT_IPS
                        && Math.abs(robot.getVelocity().omega) <= ANGULAR_REST_RAD_S;
                if (elapsed >= ACQUIRE_TIMEOUT_S) {
                    finish("FAIL: bounded heading acquisition failed");
                } else if (acquisition.sample(elapsed, acquired, ACQUIRE_DWELL_S)) {
                    acquisitionTime = elapsed;
                    holdPose = pose;
                    holding = true;
                    holdStart = now;
                    elapsed = 0;
                    samples.sample(target, actual);
                    maxSpeed = speed;
                    result = "MEASURE 5.000 s (SOTM ON, raw velocity)";
                }
            } else {
                drift = pose.distance(holdPose);
                maxDrift = Math.max(maxDrift, drift);
                maxSpeed = Math.max(maxSpeed, speed);
                maxSampleGap = Math.max(maxSampleGap, elapsed - lastSampleTime);
                // The requested window is exactly 5 s; the first later tick witnesses its end.
                if (elapsed <= HOLD_S) {
                    samples.sample(target, actual);
                    lastSampleTime = elapsed;
                }
                if (elapsed >= HOLD_S) {
                    boolean pass = ShooterAimMetrics.windowCovered(lastSampleTime, elapsed, maxSampleGap, HOLD_S, MAX_SAMPLE_GAP_S)
                            && headingGate && ShooterAimMetrics.holdPass(samples, lastSampleTime, HOLD_S - MAX_SAMPLE_GAP_S,
                            maxDrift, maxSpeed, DRIFT_LIMIT_IN, SPEED_LIMIT_IPS,
                            Constants.TeleOp.AutoFire.HEADING_TOLERANCE_DEG_DEFAULT);
                    finish(pass ? "PASS: full 5 s, stationary, every error within 3 deg"
                            : "FAIL: heading/drift/speed excursion or incomplete sampling coverage");
                }
            }
            display();
            success = true;
        } finally {
            if (!success) finish("FAIL: aim/robot API did not complete");
        }
    }

    private void finish(String status) {
        done = true;
        result = status;
        try {
            if (robot != null) robot.stopEverything();
        } finally {
            if (captured) {
                Shooter.useSotm.set(savedSotm);
                org.firstinspires.ftc.teamcode.pedro.Constants.PINPOINT_CONFIG.resetMode.set(savedReset);
                org.firstinspires.ftc.teamcode.pedro.Constants.FOLLOWER_BRAKE_AT_END.set(savedBrake);
            }
        }
    }

    private void display() {
        telemetry.addData("Result", result);
        telemetry.addData("Acquisition elapsed / completion s", "%.3f / %.3f", holding ? acquisitionTime : elapsed, acquisitionTime);
        telemetry.addData("Hold window / elapsed boundary s", "%.3f / %.3f", HOLD_S, holding ? elapsed : 0);
        telemetry.addData("Last in-window sample / max sample gap s", "%.4f / %.4f", lastSampleTime, maxSampleGap);
        telemetry.addData("Target / actual / wrapped error deg", "%.3f / %.3f / %.3f",
                Math.toDegrees(target), Math.toDegrees(actual), Math.toDegrees(ShooterAimMetrics.wrappedError(target, actual)));
        telemetry.addData("Samples / circular peak-to-peak / max abs / RMS deg", "%d / %.4f / %.4f / %.4f",
                samples.count(), Math.toDegrees(samples.peakToPeak()), Math.toDegrees(samples.maxAbs), Math.toDegrees(samples.rms()));
        telemetry.addData("Drift / max drift in; speed / max speed in/s", "%.4f / %.4f; %.4f / %.4f", drift, maxDrift, speed, maxSpeed);
        telemetry.addData("Main target / actual TPS", "%.2f / %.2f", mainTarget, main);
        telemetry.addData("Counter target / actual TPS", "%.2f / %.2f", counterTarget, counter);
        telemetry.addData("Wheel gate (40 TPS) / heading gate (3 deg) / combined",
                "%s / %s / %s", wheelGate, headingGate, wheelGate && headingGate ? "OPEN" : "CLOSED");
        telemetry.addData("SOTM measurement / live", "ON / %s", Shooter.useSotm.get());
        telemetry.addLine("Aim-only / no feed / no scoring; neutral Y uses manual turn, NOT synthetic follower HOLD.");
        telemetry.addLine("PASS: 8 s acquisition bound, 0.5 s dwell; full 5 s; ALL errors <=3 deg; drift <=0.5 in, speed <=0.5 in/s.");
        telemetry.addLine("5 s sampled window; stop on first boundary tick; FAIL if sample gap/boundary delay >0.1 s. No filtering/interpolation.");
        telemetry.addLine("Final numerical readout frozen with all outputs OFF. NaN = not measured/reached.");
        telemetry.update();
    }

    @Override public void stop() {
        if (!done) finish("FAIL: STOP before full measurement");
        else finish(result);
        display();
    }
}
