package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.config.ConfigVar;
import com.pedropathing.config.Configuration;
import com.pedropathing.revhub.localizers.PinpointConfig;

import java.util.Objects;

/**
 * Tuning values for {@link EKFPinpoint}. Linear values use the global distance unit selected in
 * {@link PinpointConfig}; angular values are in radians.
 */
public class EKFPinpointConfig extends PinpointConfig {
    public final ConfigVar<Double> initialPositionStandardDeviation = ConfigVar.of(0.5);
    public final ConfigVar<Double> initialHeadingStandardDeviation = ConfigVar.of(Math.toRadians(5.0));
    public final ConfigVar<Double> initialVelocityStandardDeviation = ConfigVar.of(24.0);
    public final ConfigVar<Double> initialAngularVelocityStandardDeviation = ConfigVar.of(2.0);
    public final ConfigVar<Double> linearAccelerationStandardDeviation = ConfigVar.of(72.0);
    public final ConfigVar<Double> angularAccelerationStandardDeviation = ConfigVar.of(8.0);
    public final ConfigVar<Double> positionMeasurementStandardDeviation = ConfigVar.of(0.1);
    public final ConfigVar<Double> headingMeasurementStandardDeviation = ConfigVar.of(Math.toRadians(1.0));

    public EKFPinpointConfig(Configuration<EKFPinpointConfig> config) {
        super(c -> {});
        Objects.requireNonNull(config, "config").configure(this);
    }
}
