package org.firstinspires.ftc.teamcode.fakeHw;

import com.qualcomm.robotcore.hardware.HardwareDevice;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/** Test-only lifecycle shared by sensors and servo controllers, not motor fakes. */
public abstract class FakeSensorDevice implements HardwareDevice {
    private boolean closed;
    private int resetCount;
    private int closeCount;

    @Override public Manufacturer getManufacturer() { return Manufacturer.Other; }
    @Override public String getDeviceName() { return getClass().getSimpleName(); }
    @Override public String getConnectionInfo() { return "fake:" + getDeviceName(); }
    @Override public int getVersion() { return 1; }
    @Override public void resetDeviceConfigurationForOpMode() { requireOpen(); resetCount++; }
    @Override public void close() { closed = true; closeCount++; }
    public final boolean isClosed() { return closed; }
    public final int getResetCount() { return resetCount; }
    public final int getCloseCount() { return closeCount; }

    protected final void requireOpen() {
        if (closed) throw new IllegalStateException(getDeviceName() + " is closed");
    }

    static double finite(double value, String name) {
        if (!Double.isFinite(value)) throw new IllegalArgumentException(name + " must be finite");
        return value;
    }

    static double clip(double value, double min, double max) {
        finite(value, "value");
        return Math.max(min, Math.min(max, value));
    }

    static <T> List<T> snapshot(List<T> values) {
        return Collections.unmodifiableList(new ArrayList<>(values));
    }
}
