package org.firstinspires.ftc.teamcode.robotDev.sim;

import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.rmsftc.utils.AllianceColor;
import com.rmsftc.utils.DriveVector;
import com.rmsftc.utils.RobotMode;
import com.rmsftc.utils.math.Pose;
import org.firstinspires.ftc.teamcode.mechanisms.Robot;
import org.firstinspires.ftc.teamcode.mechanisms.Shooter;
import org.firstinspires.ftc.teamcode.planners.CommonKt;
import org.firstinspires.ftc.teamcode.planners.Constants;

@TeleOp(name = "Sim #2 SOTM OFF vs ON Y Aim", group = "Sim diagnostics")
public class SimSotmAimTest extends OpMode {
    private static final double FORWARD_POWER = 0.15, RUN_S = 3, REST_TIMEOUT_S = 8;
    private static final double REST_TPS = 5, REST_IPS = 0.2, REST_OMEGA = 0.1, DWELL_S = 0.3;
    private static final double MOVING_IPS = 0.5, MIN_DISPLACEMENT_IN = 1, MAX_TRAVEL_IN = 15;
    private static final int MIN_MOVING_SAMPLES = 5;
    private Robot robot;
    private DcMotorEx transferMotor;
    private Pose startPose;
    private PinpointLocalizer.ResetMode savedReset;
    private boolean captured, savedSotm, savedBrake, done, runningPhase, previousSmart;
    private int phase;
    private long phaseStart;
    private ShooterAimMetrics.Dwell rest = new ShooterAimMetrics.Dwell();
    private final ShooterAimMetrics.Dwell[] settle = {new ShooterAimMetrics.Dwell(), new ShooterAimMetrics.Dwell()};
    private final ShooterAimMetrics.Heading[] moving = {new ShooterAimMetrics.Heading(), new ShooterAimMetrics.Heading()};
    private final int[] gateSamples = new int[2], commandGateSamples = new int[2], targetChanges = new int[2];
    private final boolean[] passed = new boolean[2];
    private final double[] displacement = new double[2], lastChange = {Double.NaN, Double.NaN};
    private final double[] maxSpeed = new double[2], restTime = {Double.NaN, Double.NaN};
    private final double[] phaseDuration = new double[2];
    private final String[] phaseResult = {"INCOMPLETE", "INCOMPLETE"};
    private double elapsed, target, actual, speed, mainTarget, counterTarget, main, counter, transferPower;
    private double lastMainTarget = Double.NaN, lastCounterTarget = Double.NaN;
    private boolean wheelGate, headingGate, commandGate, currentGate;
    private boolean targetChanged;
    private String transferMode = "IDLE";
    private String result = "INIT: RED garden shoot pose, EMPTY shooter; clear forward travel";

    @Override public void init() {
        boolean success = false;
        try {
            savedSotm = Shooter.useSotm.get();
            savedBrake = org.firstinspires.ftc.teamcode.pedro.Constants.FOLLOWER_BRAKE_AT_END.get();
            savedReset = org.firstinspires.ftc.teamcode.pedro.Constants.PINPOINT_CONFIG.resetMode.get();
            captured = true;
            CommonKt.init(AllianceColor.RED);
            startPose = new Pose(Constants.ShootPoses.garden.x(), Constants.ShootPoses.garden.y(),
                    Constants.ShootPoses.garden.heading());
            robot = new Robot(telemetry, hardwareMap, AllianceColor.RED, RobotMode.TELEOP, startPose);
            transferMotor = hardwareMap.get(DcMotorEx.class, "transfer");
            robot.setSmartIntakeControl(false);
            robot.setSmartShooterControl(false);
            robot.debug = false;
            robot.shooter.stopImmediately();
            robot.transfer.stopImmediately();
            Shooter.useSotm.set(false);
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
            phaseStart = System.nanoTime();
            result = "OFF WAIT_REST (bounded; wheels and robot must be stationary)";
            success = true;
        } finally {
            if (!success) finish("FAIL: robot start did not complete");
        }
    }

    @Override public void loop() {
        if (done) { display(); return; }
        boolean success = false;
        try {
            elapsed = (System.nanoTime() - phaseStart) / 1e9;
            if (runningPhase) {
                Shooter.useSotm.set(phase == 1);
                robot.setTeleOpDrive(new DriveVector(FORWARD_POWER, 0, 0), false, false);
                robot.setAutoAlign();
                // MainTeleOp.outtakeLoop: gates are sampled BEFORE robot.update; previous SMART
                // is required. No reverse/panic/eject/slow/manual-fast overrides in this test.
                commandGate = ShooterAimMetrics.yAutoFire(previousSmart, robot.shooter.upToSpeed(),
                        ShooterAimMetrics.headingReady(robot.autoAimerCalculator.getTargetHeading(),
                                robot.getHeading(), Constants.TeleOp.AutoFire.HEADING_TOLERANCE_DEG_DEFAULT));
                robot.setSmartShooterControl(true);
                if (commandGate) robot.transfer.shootAll();
                else robot.transfer.stop();
                previousSmart = true;
            } else {
                robot.setSmartShooterControl(false);
                robot.shooter.stopImmediately();
                robot.transfer.stopImmediately();
                robot.setTeleOpDrive(new DriveVector(0, 0, 0), false, false);
                commandGate = false;
            }
            robot.update(); // Exactly one tick, including all 11 Robot daemons/localization.
            elapsed = (System.nanoTime() - phaseStart) / 1e9;
            readMetrics();
            Pose pose = robot.getPose();
            if (!ShooterAimMetrics.finite(elapsed, target, actual, speed, mainTarget, counterTarget,
                    main, counter, transferPower, pose.x(), pose.y(), robot.getVelocity().omega)) {
                finish("FAIL: nonfinite metric");
            } else if (!runningPhase) {
                boolean atRest = Math.abs(main) <= REST_TPS && Math.abs(counter) <= REST_TPS
                        && speed <= REST_IPS && Math.abs(robot.getVelocity().omega) <= REST_OMEGA;
                if (elapsed >= REST_TIMEOUT_S) {
                    finish("FAIL: bounded wheel/robot rest wait");
                } else if (rest.sample(elapsed, atRest, DWELL_S)) {
                    restTime[phase] = elapsed;
                    // Only reset a stationary phase; identical explicit pose for OFF and ON.
                    robot.follower.stop();
                    robot.follower.localizer.setPose(new Pose(startPose.x(), startPose.y(), startPose.heading()));
                    robot.autoAimerCalculator.initializeCorrectedPose(new Pose(startPose.x(), startPose.y(), startPose.heading()));
                    Shooter.useSotm.set(phase == 1);
                    previousSmart = false;
                    runningPhase = true;
                    phaseStart = System.nanoTime();
                    elapsed = 0;
                    lastMainTarget = Double.NaN;
                    lastCounterTarget = Double.NaN;
                    result = phase == 0 ? "OFF DRIVE + Y" : "ON DRIVE + Y";
                }
            } else {
                targetChanged = mainTarget != lastMainTarget || counterTarget != lastCounterTarget;
                if (targetChanged) {
                    targetChanges[phase]++;
                    lastChange[phase] = elapsed;
                }
                lastMainTarget = mainTarget;
                lastCounterTarget = counterTarget;
                displacement[phase] = Math.max(displacement[phase], pose.distance(startPose));
                maxSpeed[phase] = Math.max(maxSpeed[phase], speed);
                if (speed > MOVING_IPS) moving[phase].sample(target, actual);
                if (speed > MOVING_IPS && currentGate) gateSamples[phase]++;
                if (speed > MOVING_IPS && commandGate) commandGateSamples[phase]++;
                settle[phase].sample(elapsed, wheelGate, DWELL_S);
                if (displacement[phase] > MAX_TRAVEL_IN) finish("FAIL: exceeded 15 in diagnostic travel bound");
                else if (elapsed >= RUN_S) endPhase();
            }
            display();
            success = true;
        } finally {
            if (!success) finish("FAIL: aim/robot API did not complete");
        }
    }

    private void readMetrics() {
        target = robot.autoAimerCalculator.getTargetHeading();
        actual = robot.getHeading();
        speed = Math.hypot(robot.getVelocity().vx, robot.getVelocity().vy);
        mainTarget = robot.shooter.mainMotor.targetTicksPerSecond;
        counterTarget = robot.shooter.counterRoller.targetTicksPerSecond;
        main = robot.shooter.mainMotor.getTicksPerSecond();
        counter = robot.shooter.counterRoller.getTicksPerSecond();
        wheelGate = robot.shooter.upToSpeed();
        headingGate = ShooterAimMetrics.headingReady(target, actual, Constants.TeleOp.AutoFire.HEADING_TOLERANCE_DEG_DEFAULT);
        currentGate = wheelGate && headingGate;
        transferMode = robot.transfer.mode().name();
        transferPower = transferMotor.getPower();
    }

    private void endPhase() {
        passed[phase] = ShooterAimMetrics.movingPass(moving[phase], displacement[phase], MIN_MOVING_SAMPLES,
                MIN_DISPLACEMENT_IN, Constants.TeleOp.AutoFire.HEADING_TOLERANCE_DEG_DEFAULT)
                && gateSamples[phase] > 0 && commandGateSamples[phase] > 0
                && Double.isFinite(settle[phase].settledAt);
        phaseDuration[phase] = elapsed;
        phaseResult[phase] = passed[phase] ? "PASS"
                : moving[phase].count() < MIN_MOVING_SAMPLES || displacement[phase] < MIN_DISPLACEMENT_IN
                ? "FAIL: no sufficient measured motion"
                : !moving[phase].within(Constants.TeleOp.AutoFire.HEADING_TOLERANCE_DEG_DEFAULT)
                ? "FAIL: moving heading error"
                : gateSamples[phase] == 0 || commandGateSamples[phase] == 0
                ? "FAIL: no simultaneous/command gates" : "FAIL: no sustained wheel settle";
        robot.setSmartShooterControl(false);
        robot.shooter.stopImmediately();
        robot.transfer.stopImmediately();
        robot.follower.stop();
        robot.follower.drivetrain.drive(new DriveVector(0, 0, 0), false);
        if (phase == 1) {
            finish(passed[0] && passed[1] ? "PASS: BOTH phases satisfy disclosed criteria"
                    : "FAIL: one/both phases failed; no improvement assumption");
        } else {
            phase = 1;
            runningPhase = false;
            previousSmart = false;
            rest = new ShooterAimMetrics.Dwell();
            phaseStart = System.nanoTime();
            result = "ON WAIT_REST: outputs OFF; bounded coast-down before stationary pose reset";
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
        telemetry.addData("Phase / elapsed s / SOTM live", "%s / %.3f / %s", phase == 0 ? "OFF" : "ON", elapsed, Shooter.useSotm.get());
        telemetry.addData("Drive command", "%.2f forward POWER (robot-centric, before production scaling), NOT measured speed", FORWARD_POWER);
        telemetry.addData("Target / actual / wrapped error deg", "%.3f / %.3f / %.3f",
                Math.toDegrees(target), Math.toDegrees(actual), Math.toDegrees(ShooterAimMetrics.wrappedError(target, actual)));
        telemetry.addData("Main target / actual / error TPS", "%.2f / %.2f / %.2f", mainTarget, main, mainTarget - main);
        telemetry.addData("Counter target / actual / error TPS", "%.2f / %.2f / %.2f", counterTarget, counter, counterTarget - counter);
        telemetry.addData("Measured translation in/s", speed);
        telemetry.addData("Wheel gate 40 TPS / heading gate 3 deg / simultaneous", "%s / %s / %s",
                wheelGate, headingGate, currentGate ? "OPEN" : "CLOSED");
        telemetry.addData("Y command gates PRE-update / transfer applied mode / power",
                "%s / %s / %.3f", commandGate ? "OPEN" : "CLOSED", transferMode, transferPower);
        telemetry.addData("Wheel targets changed this tick / phase count / last elapsed s",
                "%s / %d / %.4f", targetChanged, targetChanges[phase], lastChange[phase]);
        for (int i = 0; i < 2; i++) {
            telemetry.addData(i == 0 ? "OFF summary" : "ON summary",
                    "%s; moving n=%d max/RMS=%.3f/%.3f deg; displacement=%.3f in maxSpeed=%.3f in/s; MOVING simultaneous/command gates=%d/%d",
                    phaseResult[i],
                    moving[i].count(), Math.toDegrees(moving[i].maxAbs), Math.toDegrees(moving[i].rms()),
                    displacement[i], maxSpeed[i], gateSamples[i], commandGateSamples[i]);
            telemetry.addData(i == 0 ? "OFF timing s" : "ON timing s",
                    "run=%.3f; rest=%.3f; first dual-wheel gate=%.3f; settle completion=%.3f; current/longest dwell=%.3f/%.3f",
                    phaseDuration[i], restTime[i], settle[i].firstEntry, settle[i].settledAt, settle[i].current, settle[i].longest);
        }
        telemetry.addLine("Each phase 3 s: >=5 samples speed>0.5 in/s, >=1 in displacement, moving max error<=3 deg, >=1 MOVING simultaneous AND command gate, wheel dwell>=0.3 s.");
        telemetry.addLine("Wheel/rest dwell restarts if the sampling gap exceeds 0.1 s.");
        telemetry.addLine("OFF retains production stationary solution (no velocity compensation), while still driving.");
        telemetry.addLine("Gate OPEN is NOT scoring (no balls). 3 deg is fixed production default, not Dashboard override.");
        telemetry.addLine("Final numbers frozen; outputs OFF; live SOTM restored. NaN = not reached.");
        telemetry.update();
    }

    @Override public void stop() {
        if (!done) finish("FAIL: STOP before both phases completed");
        else finish(result);
        display();
    }
}
