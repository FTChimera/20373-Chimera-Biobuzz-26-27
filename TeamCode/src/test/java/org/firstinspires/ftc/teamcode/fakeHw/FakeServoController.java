package org.firstinspires.ftc.teamcode.fakeHw;

import com.qualcomm.robotcore.hardware.PwmControl.PwmRange;
import com.qualcomm.robotcore.hardware.ServoControllerEx;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.ServoConfigurationType;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Objects;

/** Explicit ports; positions are commanded PWM values, not measured shaft angles. */
public final class FakeServoController extends FakeSensorDevice implements ServoControllerEx {
    public static final class PositionWrite {
        public final int port;
        public final double position;
        private PositionWrite(int port, double position) { this.port = port; this.position = position; }
    }

    private final double[] positions;
    private final boolean[] enabled;
    private final PwmRange[] ranges;
    private final int[] forgetCounts;
    private final List<PositionWrite> writes = new ArrayList<>();
    private int pwmWriteCount;
    private int rangeWriteCount;

    public FakeServoController() { this(6); }
    public FakeServoController(int portCount) {
        if (portCount <= 0) throw new IllegalArgumentException("portCount must be positive");
        positions = new double[portCount];
        enabled = new boolean[portCount];
        ranges = new PwmRange[portCount];
        forgetCounts = new int[portCount];
        Arrays.fill(positions, 0.5);
        Arrays.fill(enabled, true);
        Arrays.fill(ranges, PwmRange.defaultRange);
    }

    private void port(int port) {
        requireOpen();
        if (port < 0 || port >= positions.length) throw new IllegalArgumentException("Unknown servo port " + port);
    }
    @Override public void setServoPosition(int port, double position) {
        port(port);
        positions[port] = clip(position, 0, 1);
        enabled[port] = true;
        writes.add(new PositionWrite(port, positions[port]));
    }
    @Override public double getServoPosition(int port) { port(port); return positions[port]; }
    @Override public void forgetLastKnownPosition(int port) { port(port); forgetCounts[port]++; }
    public int getForgetCount(int port) { port(port); return forgetCounts[port]; }
    public List<PositionWrite> getPositionHistory() { return snapshot(writes); }
    public int getPositionWriteCount() { return writes.size(); }
    public int getPwmWriteCount() { return pwmWriteCount; }
    public int getRangeWriteCount() { return rangeWriteCount; }
    @Override public void pwmEnable() { requireOpen(); Arrays.fill(enabled, true); pwmWriteCount++; }
    @Override public void pwmDisable() { requireOpen(); Arrays.fill(enabled, false); pwmWriteCount++; }
    @Override public PwmStatus getPwmStatus() {
        requireOpen();
        int count = 0;
        for (boolean value : enabled) if (value) count++;
        return count == 0 ? PwmStatus.DISABLED : count == enabled.length ? PwmStatus.ENABLED : PwmStatus.MIXED;
    }
    @Override public void setServoPwmRange(int port, PwmRange range) {
        port(port);
        Objects.requireNonNull(range, "range");
        finite(range.usPulseLower, "lower pulse");
        finite(range.usPulseUpper, "upper pulse");
        finite(range.usFrame, "frame");
        if (range.usPulseLower <= 0 || range.usPulseUpper <= range.usPulseLower
                || range.usFrame <= range.usPulseUpper) throw new IllegalArgumentException("Invalid PWM range");
        ranges[port] = range;
        rangeWriteCount++;
    }
    @Override public PwmRange getServoPwmRange(int port) { port(port); return ranges[port]; }
    @Override public void setServoPwmEnable(int port) { port(port); enabled[port] = true; pwmWriteCount++; }
    @Override public void setServoPwmDisable(int port) { port(port); enabled[port] = false; pwmWriteCount++; }
    @Override public boolean isServoPwmEnabled(int port) { port(port); return enabled[port]; }
    @Override public void setServoType(int port, ServoConfigurationType type) {
        port(port);
        Objects.requireNonNull(type, "type");
        setServoPwmRange(port, new PwmRange(type.getUsPulseLower(), type.getUsPulseUpper(), type.getUsFrame()));
    }
    @Override public void setPulseWidth(int port, double microseconds) {
        port(port);
        finite(microseconds, "pulse width");
        PwmRange range = ranges[port];
        setServoPosition(port, (clip(microseconds, range.usPulseLower, range.usPulseUpper)
                - range.usPulseLower) / (range.usPulseUpper - range.usPulseLower));
    }
    @Override public double getPulseWidth(int port) {
        port(port);
        return ranges[port].usPulseLower + positions[port] * (ranges[port].usPulseUpper - ranges[port].usPulseLower);
    }
    @Override public void resetDeviceConfigurationForOpMode() {
        super.resetDeviceConfigurationForOpMode();
        Arrays.fill(ranges, PwmRange.defaultRange);
        Arrays.fill(enabled, true);
    }
}
