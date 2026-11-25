package org.firstinspires.ftc.teamcode.fakeHw;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.AnalogInputController;
import com.qualcomm.robotcore.util.SerialNumber;

/** A real SDK AnalogInput backed by a one-port controller; no Android context needed. */
public final class FakeAnalogInput extends AnalogInput {
    public static final class Controller extends FakeSensorDevice implements AnalogInputController {
        private final int port;
        private final double maxVoltage;
        private double voltage;
        public Controller(int port, double maxVoltage) {
            if (port < 0) throw new IllegalArgumentException("port must be nonnegative");
            finite(maxVoltage, "maxVoltage");
            if (maxVoltage <= 0) throw new IllegalArgumentException("maxVoltage must be positive");
            this.port = port;
            this.maxVoltage = maxVoltage;
        }
        public void setVoltage(double voltage) {
            requireOpen();
            finite(voltage, "voltage");
            if (voltage < 0 || voltage > maxVoltage) throw new IllegalArgumentException("voltage outside controller range");
            this.voltage = voltage;
        }
        @Override public double getAnalogInputVoltage(int port) {
            requireOpen();
            if (this.port != port) throw new IllegalArgumentException("Unknown analog port " + port);
            return voltage;
        }
        @Override public double getMaxAnalogInputVoltage() { requireOpen(); return maxVoltage; }
        @Override public SerialNumber getSerialNumber() {
            throw new UnsupportedOperationException("Fake analog controller has no physical serial number");
        }
    }
    private final Controller controller;
    public FakeAnalogInput() { this(0, 3.3); }
    public FakeAnalogInput(int port, double maxVoltage) { this(new Controller(port, maxVoltage)); }
    public FakeAnalogInput(Controller controller) {
        super(java.util.Objects.requireNonNull(controller, "controller"), controller.port);
        this.controller = controller;
    }
    public Controller getController() { return controller; }
    public void setVoltage(double voltage) { controller.setVoltage(voltage); }
    @Override public String getDeviceName() { return "FakeAnalogInput"; }
    @Override public void resetDeviceConfigurationForOpMode() { controller.resetDeviceConfigurationForOpMode(); }
    @Override public void close() { controller.close(); }
}
