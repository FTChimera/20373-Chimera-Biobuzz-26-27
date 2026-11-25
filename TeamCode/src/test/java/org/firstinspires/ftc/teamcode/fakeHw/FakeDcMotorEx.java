package org.firstinspires.ftc.teamcode.fakeHw;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.MotorControlAlgorithm;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.EnumMap;
import java.util.List;
import java.util.Objects;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Rotation;

/**
 * Deterministic shaft plant, stepped only explicitly. Units are ticks, seconds and amps.
 * PID values are configuration, not a simulation of hub firmware: velocity control tracks a
 * clipped setpoint with first-order lag. Load reduces available speed; stall/fault freeze the
 * shaft. BRAKE and FLOAT use different decay constants. No gravity or mechanical backlash.
 */
public final class FakeDcMotorEx extends FakeSensorDevice implements DcMotorEx, FakeSteppable {
    /** Arguments contain only immutable scalar values/enums; snapshots never alias SDK configs. */
    public static final class Call {
        public final String operation;
        public final List<Object> arguments;
        public final boolean write;
        private Call(String operation, boolean write, Object... arguments) {
            this.operation = operation;
            this.write = write;
            this.arguments = snapshot(Arrays.asList(arguments));
        }
    }

    private final FakeDcMotorController controller;
    private final int port;
    private final List<Call> calls = new ArrayList<>();
    private final List<Call> writes = new ArrayList<>();
    private final List<Double> powers = new ArrayList<>();
    private final List<Double> velocities = new ArrayList<>();
    private final EnumMap<RunMode, PIDFCoefficients> pid = new EnumMap<>(RunMode.class);
    private MotorConfigurationType motorType;
    private Direction direction = Direction.FORWARD;
    private RunMode mode = RunMode.RUN_WITHOUT_ENCODER;
    private ZeroPowerBehavior zero = ZeroPowerBehavior.FLOAT;
    private boolean enabled = true;
    private boolean stalled;
    private boolean faulted;
    private boolean velocityControl;
    private double shaftPosition;
    private double encoderZero;
    private double shaftVelocity;
    private double power;
    private double velocityCommand;
    private int target;
    private int tolerance = 5;
    private int appliedTolerance = 5;
    private double freeSpeed = 2800;
    private double batteryVoltage = 12;
    private double nominalVoltage = 12;
    private double timeConstant = 0.1;
    private double brakeTimeConstant = 0.025;
    private double floatTimeConstant = 0.5;
    private double load;
    private double idleCurrent = 0.2;
    private double stallCurrent = 8;
    private double currentAlert = 5;
    private Double currentOverride;
    private double elapsedSeconds;

    public FakeDcMotorEx() { this(new FakeDcMotorController(1), 0); }

    public FakeDcMotorEx(FakeDcMotorController controller, int port) {
        this.controller = Objects.requireNonNull(controller, "controller");
        this.port = port;
        motorType = new MotorConfigurationType();
        motorType.setTicksPerRev(560);
        motorType.setMaxRPM(300);
        motorType.setAchieveableMaxRPMFraction(1);
        motorType.setOrientation(Rotation.CW);
        pid.put(RunMode.RUN_USING_ENCODER, new PIDFCoefficients());
        pid.put(RunMode.RUN_TO_POSITION, new PIDFCoefficients());
        controller.attach(port, this);
    }

    private void record(String operation, boolean write, Object... arguments) {
        requireOpen();
        controller.checkOpen();
        Call call = new Call(operation, write, arguments);
        calls.add(call);
        if (write) writes.add(call);
    }

    public List<Call> getCallHistory() { return snapshot(calls); }
    public List<Call> getWriteHistory() { return snapshot(writes); }
    public int getCallCount() { return calls.size(); }
    public int getWriteCount() { return writes.size(); }
    public int getCallCount(String operation) {
        int count = 0;
        for (Call call : calls) if (call.operation.equals(operation)) count++;
        return count;
    }
    public int getWriteCount(String operation) {
        int count = 0;
        for (Call call : writes) if (call.operation.equals(operation)) count++;
        return count;
    }
    public List<Double> getPowerHistory() { return snapshot(powers); }
    public List<Double> getVelocityHistory() { return snapshot(velocities); }
    public int getPowerWriteCount() { return powers.size(); }
    public int getVelocityWriteCount() { return velocities.size(); }

    private int sign() {
        boolean reverse = direction == Direction.REVERSE;
        if (motorType.getOrientation() == Rotation.CCW) reverse = !reverse;
        return reverse ? -1 : 1;
    }

    @Override public FakeDcMotorController getController() { record("getController", false); return controller; }
    @Override public int getPortNumber() { record("getPortNumber", false); return port; }
    @Override public void setDirection(Direction value) {
        Objects.requireNonNull(value, "direction");
        record("setDirection", true, value);
        direction = value;
    }
    @Override public Direction getDirection() { record("getDirection", false); return direction; }
    @Override public void setPower(double value) {
        value = clip(value, -1, 1);
        record("setPower", true, value);
        powers.add(value);
        rawPower(mode == RunMode.RUN_TO_POSITION ? Math.abs(value) : sign() * value);
    }
    void rawPower(double value) {
        power = clip(value, -1, 1);
        velocityControl = false;
        enabled = true;
    }
    double rawPower() { requireOpen(); return power; }
    @Override public double getPower() {
        record("getPower", false);
        return mode == RunMode.RUN_TO_POSITION ? Math.abs(power) : sign() * power;
    }
    public double getPhysicalPower() { record("getPhysicalPower", false); return power; }
    @Override public void setVelocity(double value) {
        finite(value, "velocity");
        record("setVelocity", true, value);
        velocities.add(value);
        rawVelocity(sign() * value);
    }
    @Override public void setVelocity(double value, AngleUnit unit) {
        double ticks = angularToTicks(value, unit);
        record("setVelocity", true, value, unit);
        velocities.add(ticks);
        rawVelocity(sign() * ticks);
    }
    void rawVelocity(double value) {
        velocityCommand = finite(value, "velocity");
        velocityControl = true;
        enabled = true;
        // REV velocity commands implicitly select the encoder velocity mode.
        if (mode != RunMode.RUN_TO_POSITION) mode = RunMode.RUN_USING_ENCODER;
    }
    double rawVelocity() { requireOpen(); return shaftVelocity; }
    @Override public double getVelocity() { record("getVelocity", false); return sign() * shaftVelocity; }
    @Override public double getVelocity(AngleUnit unit) {
        double value = ticksToAngular(sign() * shaftVelocity, unit);
        record("getVelocity", false, unit);
        return value;
    }
    double angularToTicks(double value, AngleUnit unit) {
        Objects.requireNonNull(unit, "unit");
        finite(value, "angular velocity");
        double ticks = positive(motorType.getTicksPerRev(), "ticks per revolution");
        return finite((unit == AngleUnit.DEGREES ? value / 360 : value / (2 * Math.PI)) * ticks, "velocity");
    }
    double ticksToAngular(double value, AngleUnit unit) {
        Objects.requireNonNull(unit, "unit");
        double ticks = positive(motorType.getTicksPerRev(), "ticks per revolution");
        return value / ticks * (unit == AngleUnit.DEGREES ? 360 : 2 * Math.PI);
    }
    @Override public MotorConfigurationType getMotorType() {
        record("getMotorType", false);
        return motorType.clone();
    }
    @Override public void setMotorType(MotorConfigurationType value) {
        Objects.requireNonNull(value, "motor type");
        nonnegative(value.getTicksPerRev(), "ticks per revolution");
        nonnegative(value.getMaxRPM(), "max RPM");
        nonnegative(value.getAchieveableMaxRPMFraction(), "achievable RPM fraction");
        Objects.requireNonNull(value.getOrientation(), "motor orientation");
        double speed = finite(value.getAchieveableMaxTicksPerSecond(), "free speed");
        record("setMotorType", true, value.getTicksPerRev(), value.getMaxRPM(),
                value.getAchieveableMaxRPMFraction(), value.getOrientation());
        motorType = value.clone();
        // The SDK's unspecified motor type has no usable speed or encoder conversion.
        if (speed > 0) freeSpeed = speed;
    }
    @Override public void setMode(RunMode value) {
        value = Objects.requireNonNull(value, "mode").migrate();
        record("setMode", true, value);
        if (mode == value) return;
        mode = value;
        velocityControl = false;
        velocityCommand = 0;
        // REV reissues cached power across mode changes, including encoder resets.
        if (value != RunMode.STOP_AND_RESET_ENCODER) enabled = true;
        if (value == RunMode.STOP_AND_RESET_ENCODER) {
            encoderZero = shaftPosition;
            shaftVelocity = 0;
            target = 0;
        }
    }
    @Override public RunMode getMode() { record("getMode", false); return mode; }
    @Override public void setZeroPowerBehavior(ZeroPowerBehavior value) {
        Objects.requireNonNull(value, "zero power behavior");
        if (value == ZeroPowerBehavior.UNKNOWN) throw new IllegalArgumentException("UNKNOWN zero power behavior");
        record("setZeroPowerBehavior", true, value);
        zero = value;
    }
    @Override public ZeroPowerBehavior getZeroPowerBehavior() { record("getZeroPowerBehavior", false); return zero; }
    @Override public void setPowerFloat() {
        record("setPowerFloat", true);
        setZeroPowerBehavior(ZeroPowerBehavior.FLOAT);
        setPower(0);
    }
    @Override public boolean getPowerFloat() { record("getPowerFloat", false); return zero == ZeroPowerBehavior.FLOAT && power == 0; }
    @Override public void setTargetPosition(int value) {
        record("setTargetPosition", true, value);
        rawTarget(sign() * value);
    }
    void rawTarget(int value) { rawTarget(value, tolerance); }
    void rawTarget(int value, int targetTolerance) {
        if (targetTolerance < 0) throw new IllegalArgumentException("tolerance must be nonnegative");
        target = value;
        appliedTolerance = targetTolerance;
    }
    int rawTarget() { requireOpen(); return target; }
    @Override public int getTargetPosition() { record("getTargetPosition", false); return sign() * target; }
    int rawPosition() { requireOpen(); return (int) Math.round(shaftPosition - encoderZero); }
    @Override public int getCurrentPosition() { record("getCurrentPosition", false); return sign() * rawPosition(); }
    public double getPhysicalPosition() { record("getPhysicalPosition", false); return shaftPosition; }
    public double getExactPosition() { record("getExactPosition", false); return sign() * (shaftPosition - encoderZero); }
    @Override public boolean isBusy() {
        record("isBusy", false);
        return mode == RunMode.RUN_TO_POSITION && Math.abs((double) target - rawPosition()) > appliedTolerance;
    }
    @Override public void setTargetPositionTolerance(int value) {
        if (value < 0) throw new IllegalArgumentException("tolerance must be nonnegative");
        record("setTargetPositionTolerance", true, value);
        tolerance = value;
    }
    @Override public int getTargetPositionTolerance() { record("getTargetPositionTolerance", false); return tolerance; }
    @Override public void setMotorEnable() { record("setMotorEnable", true); enabled = true; }
    @Override public void setMotorDisable() { record("setMotorDisable", true); enabled = false; }
    @Override public boolean isMotorEnabled() { record("isMotorEnabled", false); return enabled; }

    private RunMode pidMode(RunMode value) {
        value = Objects.requireNonNull(value, "mode").migrate();
        if (!pid.containsKey(value)) throw new UnsupportedOperationException("PID is unsupported in " + value);
        return value;
    }
    @Override public void setPIDCoefficients(RunMode value, PIDCoefficients coefficients) {
        Objects.requireNonNull(coefficients, "coefficients");
        setPIDFCoefficients(value, new PIDFCoefficients(coefficients));
        record("setPIDCoefficients", true, value.migrate(), coefficients.p, coefficients.i, coefficients.d);
    }
    @Override public void setPIDFCoefficients(RunMode value, PIDFCoefficients coefficients) {
        value = pidMode(value);
        Objects.requireNonNull(coefficients, "coefficients");
        finite(coefficients.p, "p"); finite(coefficients.i, "i");
        finite(coefficients.d, "d"); finite(coefficients.f, "f");
        Objects.requireNonNull(coefficients.algorithm, "algorithm");
        if (coefficients.algorithm == MotorControlAlgorithm.Unknown) {
            throw new UnsupportedOperationException("Unknown motor control algorithm");
        }
        if (coefficients.algorithm == MotorControlAlgorithm.LegacyPID && coefficients.f != 0) {
            throw new UnsupportedOperationException("Legacy PID requires a zero feed-forward term");
        }
        if (value == RunMode.RUN_TO_POSITION && coefficients.algorithm != MotorControlAlgorithm.LegacyPID
                && (coefficients.i != 0 || coefficients.d != 0 || coefficients.f != 0)) {
            throw new UnsupportedOperationException("RUN_TO_POSITION PIDF supports only the proportional term");
        }
        record("setPIDFCoefficients", true, value, coefficients.p, coefficients.i,
                coefficients.d, coefficients.f, coefficients.algorithm);
        pid.put(value, new PIDFCoefficients(coefficients));
    }
    @Override public PIDCoefficients getPIDCoefficients(RunMode value) {
        value = pidMode(value);
        record("getPIDCoefficients", false, value);
        PIDFCoefficients c = pid.get(value);
        return new PIDCoefficients(c.p, c.i, c.d);
    }
    @Override public PIDFCoefficients getPIDFCoefficients(RunMode value) {
        value = pidMode(value);
        record("getPIDFCoefficients", false, value);
        return new PIDFCoefficients(pid.get(value));
    }
    @Override public void setVelocityPIDFCoefficients(double p, double i, double d, double f) {
        setPIDFCoefficients(RunMode.RUN_USING_ENCODER, new PIDFCoefficients(p, i, d, f, MotorControlAlgorithm.PIDF));
        record("setVelocityPIDFCoefficients", true, p, i, d, f);
    }
    @Override public void setPositionPIDFCoefficients(double p) {
        setPIDFCoefficients(RunMode.RUN_TO_POSITION, new PIDFCoefficients(p, 0, 0, 0, MotorControlAlgorithm.PIDF));
        record("setPositionPIDFCoefficients", true, p);
    }
    private double availableSpeed() { return freeSpeed * batteryVoltage / nominalVoltage * (1 - load); }
    private double commandedSpeed() {
        double available = availableSpeed();
        double command;
        if (velocityControl) command = clip(velocityCommand, -available, available);
        else if (mode == RunMode.RUN_WITHOUT_ENCODER) command = power * available;
        else command = clip(power * freeSpeed, -available, available);
        if (mode == RunMode.RUN_TO_POSITION) {
            double error = target - (shaftPosition - encoderZero);
            return Math.abs(error) <= appliedTolerance ? 0 : Math.copySign(Math.abs(command), error);
        }
        return command;
    }
    private double amps() {
        if (currentOverride != null) return currentOverride;
        if (!enabled || faulted || mode == RunMode.STOP_AND_RESET_ENCODER) return 0;
        double effort = velocityControl ? Math.min(1, Math.abs(velocityCommand) / freeSpeed) : Math.abs(power);
        if (mode == RunMode.RUN_TO_POSITION && Math.abs(target - (shaftPosition - encoderZero)) <= appliedTolerance) effort = 0;
        if (effort == 0 || batteryVoltage == 0) return 0;
        double speedRatio = Math.min(1, Math.abs(shaftVelocity) / Math.max(1e-12, freeSpeed * batteryVoltage / nominalVoltage));
        return (idleCurrent + (stallCurrent - idleCurrent) * (stalled ? 1 : Math.max(load, 1 - speedRatio)))
                * effort * batteryVoltage / nominalVoltage;
    }
    @Override public double getCurrent(CurrentUnit unit) {
        Objects.requireNonNull(unit, "unit");
        record("getCurrent", false, unit);
        return unit.convert(amps(), CurrentUnit.AMPS);
    }
    @Override public double getCurrentAlert(CurrentUnit unit) {
        Objects.requireNonNull(unit, "unit");
        record("getCurrentAlert", false, unit);
        return unit.convert(currentAlert, CurrentUnit.AMPS);
    }
    @Override public void setCurrentAlert(double value, CurrentUnit unit) {
        Objects.requireNonNull(unit, "unit");
        value = nonnegative(unit.toAmps(value), "current alert");
        record("setCurrentAlert", true, value, CurrentUnit.AMPS);
        currentAlert = value;
    }
    @Override public boolean isOverCurrent() { record("isOverCurrent", false); return amps() > currentAlert; }

    private static double nonnegative(double value, String name) {
        finite(value, name);
        if (value < 0) throw new IllegalArgumentException(name + " must be nonnegative");
        return value;
    }
    private static double positive(double value, String name) {
        finite(value, name);
        if (value <= 0) throw new IllegalArgumentException(name + " must be positive");
        return value;
    }
    public void setFreeSpeedTicksPerSecond(double value) { positive(value, "free speed"); record("setFreeSpeedTicksPerSecond", true, value); freeSpeed = value; }
    public double getFreeSpeedTicksPerSecond() { record("getFreeSpeedTicksPerSecond", false); return freeSpeed; }
    public void setBatteryVoltage(double value) { nonnegative(value, "battery voltage"); record("setBatteryVoltage", true, value); batteryVoltage = value; }
    public double getBatteryVoltage() { record("getBatteryVoltage", false); return batteryVoltage; }
    public void setNominalVoltage(double value) { positive(value, "nominal voltage"); record("setNominalVoltage", true, value); nominalVoltage = value; }
    public double getNominalVoltage() { record("getNominalVoltage", false); return nominalVoltage; }
    public void setTimeConstant(double value) { positive(value, "time constant"); record("setTimeConstant", true, value); timeConstant = value; }
    public double getTimeConstant() { record("getTimeConstant", false); return timeConstant; }
    public void setZeroPowerTimeConstants(double brake, double floating) {
        positive(brake, "brake time constant"); positive(floating, "float time constant");
        record("setZeroPowerTimeConstants", true, brake, floating);
        brakeTimeConstant = brake; floatTimeConstant = floating;
    }
    public double getBrakeTimeConstant() { record("getBrakeTimeConstant", false); return brakeTimeConstant; }
    public double getFloatTimeConstant() { record("getFloatTimeConstant", false); return floatTimeConstant; }
    public void setLoad(double value) {
        finite(value, "load");
        if (value < 0 || value > 1) throw new IllegalArgumentException("load must be in [0,1]");
        record("setLoad", true, value); load = value;
    }
    public double getLoad() { record("getLoad", false); return load; }
    public void setStalled(boolean value) { record("setStalled", true, value); stalled = value; if (value) shaftVelocity = 0; }
    public boolean isStalled() { record("isStalled", false); return stalled; }
    public void setFaulted(boolean value) { record("setFaulted", true, value); faulted = value; if (value) shaftVelocity = 0; }
    public boolean isFaulted() { record("isFaulted", false); return faulted; }
    public void setCurrentModel(double idleAmps, double stallAmps) {
        nonnegative(idleAmps, "idle current"); nonnegative(stallAmps, "stall current");
        if (stallAmps < idleAmps) throw new IllegalArgumentException("stall current must be at least idle current");
        record("setCurrentModel", true, idleAmps, stallAmps); idleCurrent = idleAmps; stallCurrent = stallAmps;
    }
    public void setCurrentAmps(double value) { nonnegative(value, "current"); record("setCurrentAmps", true, value); currentOverride = value; }
    public void clearCurrentOverride() { record("clearCurrentOverride", true); currentOverride = null; }
    /** Encoder stimulus is logical and does not change the reset origin. */
    public void setEncoderPosition(double ticks) {
        finite(ticks, "encoder position"); record("setEncoderPosition", true, ticks);
        shaftPosition = encoderZero + sign() * ticks;
    }
    public void addEncoderTicks(double ticks) {
        finite(ticks, "encoder delta"); record("addEncoderTicks", true, ticks);
        shaftPosition += sign() * ticks;
    }
    public void setMeasuredVelocity(double ticksPerSecond) {
        finite(ticksPerSecond, "measured velocity"); record("setMeasuredVelocity", true, ticksPerSecond);
        shaftVelocity = sign() * ticksPerSecond;
    }
    public double getElapsedSeconds() { return elapsedSeconds; }

    @Override public void advance(double seconds) {
        nonnegative(seconds, "seconds");
        finite(elapsedSeconds + seconds, "total elapsed seconds");
        record("advance", false, seconds);
        if (seconds == 0) return;
        if (faulted || stalled || mode == RunMode.STOP_AND_RESET_ENCODER) {
            shaftVelocity = 0;
            elapsedSeconds += seconds;
            return;
        }
        double desired = finite(enabled ? commandedSpeed() : 0, "commanded speed");
        if (mode == RunMode.RUN_TO_POSITION && enabled
                && Math.abs(target - (shaftPosition - encoderZero)) <= appliedTolerance) {
            shaftVelocity = 0;
            elapsedSeconds += seconds;
            return;
        }
        double tau = desired == 0 ? (enabled && zero == ZeroPowerBehavior.BRAKE
                ? brakeTimeConstant : floatTimeConstant) : timeConstant;
        double oneMinusDecay = -Math.expm1(-seconds / tau);
        double delta = desired * seconds + (shaftVelocity - desired) * tau * oneMinusDecay;
        double next = shaftPosition + delta;
        double nextVelocity = shaftVelocity + (desired - shaftVelocity) * oneMinusDecay;
        if (mode == RunMode.RUN_TO_POSITION && enabled && desired != 0) {
            double error = target - (shaftPosition - encoderZero);
            double boundary = encoderZero + target - Math.copySign(appliedTolerance, error);
            if ((error > 0 && next >= boundary) || (error < 0 && next <= boundary)) {
                next = boundary;
                nextVelocity = 0;
            }
        }
        finite(next, "integrated encoder position");
        finite(nextVelocity, "integrated velocity");
        shaftPosition = next;
        shaftVelocity = nextVelocity;
        elapsedSeconds += seconds;
    }

    @Override public void resetDeviceConfigurationForOpMode() {
        record("resetDeviceConfigurationForOpMode", true);
        super.resetDeviceConfigurationForOpMode();
        direction = Direction.FORWARD;
        power = 0; velocityCommand = 0; velocityControl = false;
        mode = RunMode.RUN_WITHOUT_ENCODER;
        zero = ZeroPowerBehavior.FLOAT;
        enabled = true;
    }
    @Override public void close() {
        if (!isClosed()) {
            record("close", true);
            power = 0; velocityCommand = 0; velocityControl = false;
            zero = ZeroPowerBehavior.FLOAT; enabled = false;
        }
        super.close();
    }
}
