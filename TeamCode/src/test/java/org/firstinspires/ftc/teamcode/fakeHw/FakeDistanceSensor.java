package org.firstinspires.ftc.teamcode.fakeHw;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import java.util.Objects;

public class FakeDistanceSensor extends FakeSensorDevice implements DistanceSensor {
    private double meters;
    private boolean outOfRange;
    public FakeDistanceSensor() { }
    public FakeDistanceSensor(double distance, DistanceUnit unit) { setDistance(distance, unit); }
    public final void setDistance(double distance, DistanceUnit unit) {
        requireOpen();
        Objects.requireNonNull(unit, "unit");
        finite(distance, "distance");
        if (distance < 0) throw new IllegalArgumentException("distance must be nonnegative");
        double converted = unit.toMeters(distance);
        finite(converted, "distance in meters");
        meters = converted;
        outOfRange = false;
    }
    public final void setOutOfRange(boolean outOfRange) { requireOpen(); this.outOfRange = outOfRange; }
    @Override public double getDistance(DistanceUnit unit) {
        requireOpen();
        Objects.requireNonNull(unit, "unit");
        return outOfRange ? distanceOutOfRange : finite(unit.fromMeters(meters), "converted distance");
    }
}
