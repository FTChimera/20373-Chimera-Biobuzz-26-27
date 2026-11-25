package org.firstinspires.ftc.teamcode.fakeHw;

import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;
import com.qualcomm.robotcore.hardware.DcMotorControllerEx;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Objects;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

/**
 * Raw (direction-independent) controller ports for real DcMotorImplEx wrappers.
 * Not FakeSteppable: register getMotor(port), the sole plant owner, in the fake map.
 * A port is created lazily or explicitly attached, never replaced by a second motor.
 */
public final class FakeDcMotorController extends FakeSensorDevice implements DcMotorControllerEx {
    public static final class Call {
        public final String operation;
        public final int port;
        public final boolean write;
        public final List<Object> arguments;
        private Call(String operation, int port, boolean write, Object... arguments) {
            this.operation = operation; this.port = port; this.write = write;
            this.arguments = snapshot(Arrays.asList(arguments));
        }
    }
    private final FakeDcMotorEx[] motors;
    private final List<Call> calls = new ArrayList<>();
    private final List<Call> writes = new ArrayList<>();

    public FakeDcMotorController() { this(4); }
    public FakeDcMotorController(int portCount) {
        if (portCount <= 0) throw new IllegalArgumentException("portCount must be positive");
        motors = new FakeDcMotorEx[portCount];
    }
    void checkOpen() { requireOpen(); }
    private void checkPort(int port) {
        requireOpen();
        if (port < 0 || port >= motors.length) throw new IllegalArgumentException("Unknown motor port " + port);
    }
    void attach(int port, FakeDcMotorEx motor) {
        checkPort(port);
        if (motors[port] != null) throw new IllegalStateException("Motor port already attached: " + port);
        motors[port] = Objects.requireNonNull(motor, "motor");
    }
    public FakeDcMotorEx getMotor(int port) {
        checkPort(port);
        if (motors[port] == null) new FakeDcMotorEx(this, port);
        if (motors[port].isClosed()) throw new IllegalStateException("Motor port is closed: " + port);
        return motors[port];
    }
    private FakeDcMotorEx call(String operation, int port, boolean write, Object... arguments) {
        FakeDcMotorEx motor = getMotor(port);
        Call call = new Call(operation, port, write, arguments);
        calls.add(call);
        if (write) writes.add(call);
        return motor;
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
    @Override public void setMotorType(int port, MotorConfigurationType type) {
        Objects.requireNonNull(type, "type");
        getMotor(port).setMotorType(type);
        call("setMotorType", port, true, type.getTicksPerRev(), type.getMaxRPM(),
                type.getAchieveableMaxRPMFraction(), type.getOrientation());
    }
    @Override public MotorConfigurationType getMotorType(int port) { return call("getMotorType", port, false).getMotorType(); }
    @Override public void setMotorMode(int port, RunMode mode) {
        getMotor(port).setMode(mode);
        call("setMotorMode", port, true, mode.migrate());
    }
    @Override public RunMode getMotorMode(int port) { return call("getMotorMode", port, false).getMode(); }
    @Override public void setMotorPower(int port, double power) {
        power = clip(power, -1, 1);
        call("setMotorPower", port, true, power).rawPower(power);
    }
    @Override public double getMotorPower(int port) { return call("getMotorPower", port, false).rawPower(); }
    @Override public boolean isBusy(int port) { return call("isBusy", port, false).isBusy(); }
    @Override public void setMotorZeroPowerBehavior(int port, ZeroPowerBehavior behavior) {
        getMotor(port).setZeroPowerBehavior(behavior);
        call("setMotorZeroPowerBehavior", port, true, behavior);
    }
    @Override public ZeroPowerBehavior getMotorZeroPowerBehavior(int port) { return call("getMotorZeroPowerBehavior", port, false).getZeroPowerBehavior(); }
    @Override public boolean getMotorPowerFloat(int port) { return call("getMotorPowerFloat", port, false).getPowerFloat(); }
    @Override public void setMotorTargetPosition(int port, int position) {
        setMotorTargetPosition(port, position, 5);
    }
    @Override public void setMotorTargetPosition(int port, int position, int tolerance) {
        if (tolerance < 0) throw new IllegalArgumentException("tolerance must be nonnegative");
        call("setMotorTargetPosition", port, true, position, tolerance).rawTarget(position, tolerance);
    }
    @Override public int getMotorTargetPosition(int port) { return call("getMotorTargetPosition", port, false).rawTarget(); }
    @Override public int getMotorCurrentPosition(int port) { return call("getMotorCurrentPosition", port, false).rawPosition(); }
    @Override public void resetDeviceConfigurationForOpMode(int port) { call("resetDeviceConfigurationForOpMode", port, true).resetDeviceConfigurationForOpMode(); }
    @Override public void setMotorEnable(int port) { call("setMotorEnable", port, true).setMotorEnable(); }
    @Override public void setMotorDisable(int port) { call("setMotorDisable", port, true).setMotorDisable(); }
    @Override public boolean isMotorEnabled(int port) { return call("isMotorEnabled", port, false).isMotorEnabled(); }
    @Override public void setMotorVelocity(int port, double velocity) {
        finite(velocity, "velocity");
        call("setMotorVelocity", port, true, velocity).rawVelocity(velocity);
    }
    @Override public void setMotorVelocity(int port, double velocity, AngleUnit unit) {
        double ticks = getMotor(port).angularToTicks(velocity, unit);
        call("setMotorVelocity", port, true, velocity, unit).rawVelocity(ticks);
    }
    @Override public double getMotorVelocity(int port) { return call("getMotorVelocity", port, false).rawVelocity(); }
    @Override public double getMotorVelocity(int port, AngleUnit unit) {
        Objects.requireNonNull(unit, "unit");
        FakeDcMotorEx motor = call("getMotorVelocity", port, false, unit);
        return motor.ticksToAngular(motor.rawVelocity(), unit);
    }
    @Override public void setPIDCoefficients(int port, RunMode mode, PIDCoefficients coefficients) {
        getMotor(port).setPIDCoefficients(mode, coefficients);
        call("setPIDCoefficients", port, true, mode.migrate(), coefficients.p, coefficients.i, coefficients.d);
    }
    @Override public void setPIDFCoefficients(int port, RunMode mode, PIDFCoefficients coefficients) {
        getMotor(port).setPIDFCoefficients(mode, coefficients);
        call("setPIDFCoefficients", port, true, mode.migrate(), coefficients.p, coefficients.i,
                coefficients.d, coefficients.f, coefficients.algorithm);
    }
    @Override public PIDCoefficients getPIDCoefficients(int port, RunMode mode) { return call("getPIDCoefficients", port, false, mode).getPIDCoefficients(mode); }
    @Override public PIDFCoefficients getPIDFCoefficients(int port, RunMode mode) { return call("getPIDFCoefficients", port, false, mode).getPIDFCoefficients(mode); }
    @Override public double getMotorCurrent(int port, CurrentUnit unit) { return call("getMotorCurrent", port, false, unit).getCurrent(unit); }
    @Override public double getMotorCurrentAlert(int port, CurrentUnit unit) { return call("getMotorCurrentAlert", port, false, unit).getCurrentAlert(unit); }
    @Override public void setMotorCurrentAlert(int port, double current, CurrentUnit unit) {
        getMotor(port).setCurrentAlert(current, unit);
        call("setMotorCurrentAlert", port, true, current, unit);
    }
    @Override public boolean isMotorOverCurrent(int port) { return call("isMotorOverCurrent", port, false).isOverCurrent(); }
    @Override public void resetDeviceConfigurationForOpMode() {
        super.resetDeviceConfigurationForOpMode();
        for (FakeDcMotorEx motor : motors) if (motor != null) motor.resetDeviceConfigurationForOpMode();
    }
    @Override public void close() {
        if (!isClosed()) for (FakeDcMotorEx motor : motors) if (motor != null) motor.close();
        super.close();
    }
}
