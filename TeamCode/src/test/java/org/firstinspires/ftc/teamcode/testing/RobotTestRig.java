package org.firstinspires.ftc.teamcode.testing;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.rmsftc.utils.AllianceColor;
import com.rmsftc.utils.RobotMode;
import com.rmsftc.utils.math.Pose;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.fakeHw.FakeDcMotorEx;
import org.firstinspires.ftc.teamcode.fakeHw.FakeHardwareMap;
import org.firstinspires.ftc.teamcode.fakeHw.FakeServo;
import org.firstinspires.ftc.teamcode.fakeHw.FakeVoltageSensor;
import org.firstinspires.ftc.teamcode.mechanisms.Robot;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.pedro.Follower;
import org.firstinspires.ftc.teamcode.util.FakeClock;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.function.DoubleSupplier;

/**
 * Fake electrical/odometry boundaries around real match mechanisms, Foresight and Mecanum.
 * Default speeds are an illustrative ideal plant, not measured robot tuning.
 * Advance before each OpMode tick: the plant integrates the previous loop's commands.
 */
public final class RobotTestRig implements AutoCloseable {
    public static final class TransferTransition {
        public final double seconds;
        public final double power;
        private TransferTransition(double seconds, double power) {
            this.seconds = seconds;
            this.power = power;
        }
    }
    public static final double WHEEL_FREE_SPEED = 2800;
    public final FakeHardwareMap hardwareMap = new FakeHardwareMap();
    public final FakeDcMotorEx frontLeft = new FakeDcMotorEx();
    public final FakeDcMotorEx frontRight = new FakeDcMotorEx();
    public final FakeDcMotorEx backLeft = new FakeDcMotorEx();
    public final FakeDcMotorEx backRight = new FakeDcMotorEx();
    public final FakeDcMotorEx intake = new FakeDcMotorEx();
    public final FakeDcMotorEx transfer = new FakeDcMotorEx();
    public final FakeDcMotorEx mainShooter = new FakeDcMotorEx();
    public final FakeDcMotorEx counterShooter = new FakeDcMotorEx();
    public final FakeServo deployer = new FakeServo();
    public final FakeVoltageSensor voltage = new FakeVoltageSensor(12);
    public final SimulatedMecanumLocalizer localizer;
    public final List<FakeDcMotorEx> driveMotors;
    public final List<FakeDcMotorEx> motors;
    public final FakeClock clock;
    public Robot robot;

    private final DoubleSupplier battery;
    private final PedroTimerScope timers;
    private final Map<Long, Double> pathProgress = new LinkedHashMap<>();
    private final List<Double> transferTransitions = new ArrayList<>();
    private final List<TransferTransition> timedTransferTransitions = new ArrayList<>();
    private double previousTransferPower;
    private double distanceTravelled;
    private boolean closed;

    public RobotTestRig(FakeClock clock) {
        this(clock, () -> 12, 81.5, 65.4, 4);
    }

    public RobotTestRig(FakeClock clock, DoubleSupplier battery, double forwardIps,
                        double strafeIps, double angularRadps) {
        this.clock = java.util.Objects.requireNonNull(clock);
        this.battery = java.util.Objects.requireNonNull(battery);
        driveMotors = Collections.unmodifiableList(Arrays.asList(
                frontLeft, frontRight, backLeft, backRight));
        motors = Collections.unmodifiableList(Arrays.asList(frontLeft, frontRight,
                backLeft, backRight, intake, transfer, mainShooter, counterShooter));
        String[] names = {"frontLeft", "frontRight", "backLeft", "backRight",
                "intake", "transfer", "mainShooter", "counterShooter"};
        for (int i = 0; i < names.length; i++) hardwareMap.register(names[i], motors.get(i));
        hardwareMap.register("deployer", deployer);
        hardwareMap.register("battery", voltage);
        for (FakeDcMotorEx motor : driveMotors) {
            motor.setFreeSpeedTicksPerSecond(WHEEL_FREE_SPEED);
            motor.setTimeConstant(0.04);
        }
        mainShooter.setFreeSpeedTicksPerSecond(2800);
        counterShooter.setFreeSpeedTicksPerSecond(2800);
        localizer = new SimulatedMecanumLocalizer(frontLeft, frontRight, backLeft,
                backRight, WHEEL_FREE_SPEED, forwardIps, strafeIps, angularRadps);
        timers = new PedroTimerScope(clock);
    }

    /** The only hook needed by match OpMode subclasses; construction retains real mechanisms. */
    public Robot createRobot(Telemetry telemetry, AllianceColor alliance,
                             RobotMode mode, Pose startingPose) {
        if (closed || robot != null) throw new IllegalStateException("Rig is single-use");
        applyBattery();
        robot = new Robot(telemetry, hardwareMap, alliance, mode, startingPose, clock,
                map -> new Follower(localizer, new Mecanum(map, Constants.MECANUM_CONFIG),
                        new Foresight(Constants.FORESIGHT_CONFIG)));
        return robot;
    }

    public void setDriveStalled(boolean stalled) {
        for (FakeDcMotorEx motor : driveMotors) motor.setStalled(stalled);
    }

    public void advance(double seconds) {
        if (closed) throw new IllegalStateException("Rig closed");
        if (!Double.isFinite(seconds) || seconds <= 0) {
            throw new IllegalArgumentException("Step must be finite and positive");
        }
        observe();
        applyBattery();
        double x = localizer.pose().x();
        double y = localizer.pose().y();
        for (FakeDcMotorEx motor : motors) motor.advance(seconds);
        localizer.advance(seconds);
        distanceTravelled += Math.hypot(localizer.pose().x() - x, localizer.pose().y() - y);
    }

    private void applyBattery() {
        double volts = battery.getAsDouble();
        voltage.setVoltage(volts);
        for (FakeDcMotorEx motor : motors) motor.setBatteryVoltage(volts);
    }

    /** Observe after run as well, before optional rig cleanup can hide an OpMode shutdown bug. */
    public void observe() {
        if (robot == null) return;
        long run = robot.follower.pathRunId();
        if (run > 0) {
            double progress = robot.follower.hasPathProgress() ? robot.follower.completion() : 0;
            pathProgress.merge(run, progress, Math::max);
        }
        double power = transfer.getPower();
        if (power != previousTransferPower) {
            transferTransitions.add(power);
            timedTransferTransitions.add(new TransferTransition(clock.nowSeconds(), power));
            previousTransferPower = power;
        }
    }

    public Map<Long, Double> pathProgress() {
        return Collections.unmodifiableMap(new LinkedHashMap<>(pathProgress));
    }
    public List<Double> transferTransitions() {
        return Collections.unmodifiableList(new ArrayList<>(transferTransitions));
    }
    public double distanceTravelled() { return distanceTravelled; }
    public List<TransferTransition> timedTransferTransitions() {
        return Collections.unmodifiableList(new ArrayList<>(timedTransferTransitions));
    }

    @Override public void close() {
        if (closed) return;
        try {
            if (robot != null) robot.stopEverything();
        } finally {
            closed = true;
            timers.close();
        }
    }
}
