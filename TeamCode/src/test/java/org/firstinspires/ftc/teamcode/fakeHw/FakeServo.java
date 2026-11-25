package org.firstinspires.ftc.teamcode.fakeHw;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.PwmControl;
import java.util.ArrayList;
import java.util.List;
import java.util.Objects;

public final class FakeServo extends FakeSensorDevice implements Servo, PwmControl {
    private final FakeServoController controller;
    private final int port;
    private Direction direction = Direction.FORWARD;
    private double min;
    private double max = 1;
    private final List<Double> positions = new ArrayList<>();

    public FakeServo() { this(new FakeServoController(), 0); }
    public FakeServo(FakeServoController controller, int port) {
        this.controller = Objects.requireNonNull(controller, "controller");
        controller.getServoPosition(port);
        this.port = port;
    }
    @Override public FakeServoController getController() { requireOpen(); return controller; }
    @Override public int getPortNumber() { return port; }
    @Override public void setDirection(Direction direction) {
        requireOpen();
        Objects.requireNonNull(direction, "direction");
        if (this.direction != direction) controller.forgetLastKnownPosition(port);
        this.direction = direction;
    }
    @Override public Direction getDirection() { requireOpen(); return direction; }
    @Override public void setPosition(double position) {
        requireOpen();
        double logical = clip(position, 0, 1);
        controller.setServoPosition(port, min + (direction == Direction.REVERSE ? 1 - logical : logical) * (max - min));
        positions.add(logical);
    }
    @Override public double getPosition() {
        requireOpen();
        double value = (controller.getServoPosition(port) - min) / (max - min);
        return clip(direction == Direction.REVERSE ? 1 - value : value, 0, 1);
    }
    public double getPhysicalPosition() { requireOpen(); return controller.getServoPosition(port); }
    public List<Double> getPositionHistory() { return snapshot(positions); }
    public int getPositionWriteCount() { return positions.size(); }
    @Override public void scaleRange(double min, double max) {
        requireOpen();
        min = clip(min, 0, 1);
        max = clip(max, 0, 1);
        if (min >= max) throw new IllegalArgumentException("min must be less than max");
        if (this.min != min || this.max != max) controller.forgetLastKnownPosition(port);
        this.min = min;
        this.max = max;
    }
    @Override public void resetDeviceConfigurationForOpMode() {
        super.resetDeviceConfigurationForOpMode();
        direction = Direction.FORWARD;
        min = 0;
        max = 1;
    }
    @Override public void setPwmRange(PwmRange range) { requireOpen(); controller.setServoPwmRange(port, range); }
    @Override public PwmRange getPwmRange() { requireOpen(); return controller.getServoPwmRange(port); }
    @Override public void setPwmEnable() { requireOpen(); controller.setServoPwmEnable(port); }
    @Override public void setPwmDisable() { requireOpen(); controller.setServoPwmDisable(port); }
    @Override public boolean isPwmEnabled() { requireOpen(); return controller.isServoPwmEnabled(port); }
    @Override public void setPulseWidth(double width) { requireOpen(); controller.setPulseWidth(port, width); }
    @Override public double getPulseWidth() { requireOpen(); return controller.getPulseWidth(port); }
}
