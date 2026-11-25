package org.firstinspires.ftc.teamcode.fakeHw;

import com.qualcomm.robotcore.hardware.VoltageSensor;

public final class FakeVoltageSensor extends FakeSensorDevice implements VoltageSensor {
    private double voltage;
    public FakeVoltageSensor() { this(12); }
    public FakeVoltageSensor(double voltage) { setVoltage(voltage); }
    public void setVoltage(double voltage) {
        requireOpen();
        finite(voltage, "voltage");
        if (voltage < 0) throw new IllegalArgumentException("voltage must be nonnegative");
        this.voltage = voltage;
    }
    @Override public double getVoltage() { requireOpen(); return voltage; }
}
