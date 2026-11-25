package org.firstinspires.ftc.teamcode.fakeHw;

import com.qualcomm.robotcore.hardware.CRServo;
import java.util.ArrayList;
import java.util.List;
import java.util.Objects;

public final class FakeCRServo extends FakeSensorDevice implements CRServo {
    private final FakeServoController controller;
    private final int port;
    private Direction direction = Direction.FORWARD;
    private final List<Double> powers = new ArrayList<>();

    public FakeCRServo() { this(new FakeServoController(), 0); }
    public FakeCRServo(FakeServoController controller, int port) {
        this.controller = Objects.requireNonNull(controller, "controller");
        controller.getServoPosition(port);
        this.port = port;
    }
    @Override public FakeServoController getController() { requireOpen(); return controller; }
    @Override public int getPortNumber() { return port; }
    @Override public void setDirection(Direction direction) { requireOpen(); this.direction = Objects.requireNonNull(direction, "direction"); }
    @Override public Direction getDirection() { requireOpen(); return direction; }
    @Override public void setPower(double power) {
        requireOpen();
        double logical = clip(power, -1, 1);
        controller.setServoPosition(port, ((direction == Direction.REVERSE ? -logical : logical) + 1) / 2);
        powers.add(logical);
    }
    @Override public double getPower() {
        requireOpen();
        double physical = getPhysicalPower();
        return direction == Direction.REVERSE ? -physical : physical;
    }
    public double getPhysicalPower() { requireOpen(); return 2 * controller.getServoPosition(port) - 1; }
    public List<Double> getPowerHistory() { return snapshot(powers); }
    public int getPowerWriteCount() { return powers.size(); }
    @Override public void resetDeviceConfigurationForOpMode() { super.resetDeviceConfigurationForOpMode(); direction = Direction.FORWARD; }
}
