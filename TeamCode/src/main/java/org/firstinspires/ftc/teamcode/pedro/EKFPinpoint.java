package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.localization.Localizer;
import com.pedropathing.localization.MotionState;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.pedropathing.utils.Angle;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.control.filters.Dynamics;
import org.firstinspires.ftc.teamcode.util.control.filters.ExtendedKalmanFilter;
import org.firstinspires.ftc.teamcode.util.control.filters.Sensor;

import java.util.HashMap;
import java.util.Map;
import java.util.Objects;

/**
 * A Pinpoint localizer that smooths its global pose measurements with a constant-velocity EKF.
 * The state is {@code [x, y, heading, vx, vy, omega]} in Pinpoint's configured global units.
 */
public class EKFPinpoint implements Localizer {
    private static final int X = 0;
    private static final int Y = 1;
    private static final int HEADING = 2;
    private static final int VX = 3;
    private static final int VY = 4;
    private static final int OMEGA = 5;

    private final PinpointLocalizer pinpoint;
    private final Tuning tuning;

    private ExtendedKalmanFilter filter;
    private MotionState motionState;
    private Pose rawPose;
    private long lastUpdateNanos;

    public EKFPinpoint(HardwareMap hardwareMap, EKFPinpointConfig config) {
        Objects.requireNonNull(hardwareMap, "hardwareMap");
        Objects.requireNonNull(config, "config");

        tuning = Tuning.from(config);
        pinpoint = new PinpointLocalizer(hardwareMap, config);
        initialize(pinpoint.pose());
    }

    @Override
    public void setPose(Pose pose) {
        Objects.requireNonNull(pose, "pose");
        pinpoint.setPose(pose);
        initialize(pose);
    }

    @Override
    public MotionState state() {
        return motionState;
    }

    @Override
    public void update() {
        pinpoint.update();
        rawPose = pinpoint.pose();

        long now = System.nanoTime();
        double dt = (now - lastUpdateNanos) / 1.0e9;
        filter.step(new double[]{rawPose.x(), rawPose.y(), rawPose.heading()}, dt);
        lastUpdateNanos = now;
        updateMotionState();
    }

    @Override
    public void reset() {
        pinpoint.reset();
        pinpoint.update();
        initialize(pinpoint.pose());
    }

    @Override
    public Map<String, Object> debug() {
        Map<String, Object> debug = new HashMap<>(Localizer.super.debug());
        debug.put("rawPose", rawPose);
        debug.put("covariance", filter.covariance());
        return debug;
    }

    private void initialize(Pose pose) {
        rawPose = pose;
        filter = new ExtendedKalmanFilter(
                new double[]{pose.x(), pose.y(), pose.heading(), 0.0, 0.0, 0.0},
                initialCovariance(),
                new ConstantVelocityDynamics(tuning),
                new PinpointPoseSensor(tuning)
        );
        lastUpdateNanos = System.nanoTime();
        updateMotionState();
    }

    private void updateMotionState() {
        double[] state = filter.state();
        Pose pose = new Pose(state[X], state[Y], state[HEADING]);
        Velocity velocity = new Velocity(state[VX], state[VY], state[OMEGA]);
        motionState = MotionState.ofVelocity(pose, velocity);
    }

    private double[][] initialCovariance() {
        return new double[][]{
                {square(tuning.initialPositionStandardDeviation), 0.0, 0.0, 0.0, 0.0, 0.0},
                {0.0, square(tuning.initialPositionStandardDeviation), 0.0, 0.0, 0.0, 0.0},
                {0.0, 0.0, square(tuning.initialHeadingStandardDeviation), 0.0, 0.0, 0.0},
                {0.0, 0.0, 0.0, square(tuning.initialVelocityStandardDeviation), 0.0, 0.0},
                {0.0, 0.0, 0.0, 0.0, square(tuning.initialVelocityStandardDeviation), 0.0},
                {0.0, 0.0, 0.0, 0.0, 0.0, square(tuning.initialAngularVelocityStandardDeviation)}
        };
    }

    private static double square(double value) {
        return value * value;
    }

    private static final class ConstantVelocityDynamics implements Dynamics {
        private final Tuning tuning;

        private ConstantVelocityDynamics(Tuning tuning) {
            this.tuning = tuning;
        }

        @Override
        public double[] f(double[] state, double dt) {
            return new double[]{
                    state[X] + state[VX] * dt,
                    state[Y] + state[VY] * dt,
                    Angle.normalize(state[HEADING] + state[OMEGA] * dt),
                    state[VX],
                    state[VY],
                    state[OMEGA]
            };
        }

        @Override
        public Matrix jacobianF(double[] state, double dt) {
            return new Matrix(new double[][]{
                    {1.0, 0.0, 0.0, dt, 0.0, 0.0},
                    {0.0, 1.0, 0.0, 0.0, dt, 0.0},
                    {0.0, 0.0, 1.0, 0.0, 0.0, dt},
                    {0.0, 0.0, 0.0, 1.0, 0.0, 0.0},
                    {0.0, 0.0, 0.0, 0.0, 1.0, 0.0},
                    {0.0, 0.0, 0.0, 0.0, 0.0, 1.0}
            });
        }

        @Override
        public Matrix Q(double dt) {
            double linearVariance = square(tuning.linearAccelerationStandardDeviation);
            double angularVariance = square(tuning.angularAccelerationStandardDeviation);
            double dt2 = dt * dt;
            double dt3 = dt2 * dt;
            double dt4 = dt2 * dt2;

            return new Matrix(new double[][]{
                    {dt4 * linearVariance / 4.0, 0.0, 0.0, dt3 * linearVariance / 2.0, 0.0, 0.0},
                    {0.0, dt4 * linearVariance / 4.0, 0.0, 0.0, dt3 * linearVariance / 2.0, 0.0},
                    {0.0, 0.0, dt4 * angularVariance / 4.0, 0.0, 0.0, dt3 * angularVariance / 2.0},
                    {dt3 * linearVariance / 2.0, 0.0, 0.0, dt2 * linearVariance, 0.0, 0.0},
                    {0.0, dt3 * linearVariance / 2.0, 0.0, 0.0, dt2 * linearVariance, 0.0},
                    {0.0, 0.0, dt3 * angularVariance / 2.0, 0.0, 0.0, dt2 * angularVariance}
            });
        }

        @Override
        public Matrix Q() {
            return Q(0.0);
        }

        @Override
        public double[] normalizeState(double[] state) {
            state[HEADING] = Angle.normalize(state[HEADING]);
            return state;
        }
    }

    private static final class PinpointPoseSensor implements Sensor {
        private final Tuning tuning;

        private PinpointPoseSensor(Tuning tuning) {
            this.tuning = tuning;
        }

        @Override
        public double[] h(double[] state) {
            return new double[]{state[X], state[Y], state[HEADING]};
        }

        @Override
        public double[][] jacobianH(double[] state) {
            return new double[][]{
                    {1.0, 0.0, 0.0, 0.0, 0.0, 0.0},
                    {0.0, 1.0, 0.0, 0.0, 0.0, 0.0},
                    {0.0, 0.0, 1.0, 0.0, 0.0, 0.0}
            };
        }

        @Override
        public double[][] R() {
            return new double[][]{
                    {square(tuning.positionMeasurementStandardDeviation), 0.0, 0.0},
                    {0.0, square(tuning.positionMeasurementStandardDeviation), 0.0},
                    {0.0, 0.0, square(tuning.headingMeasurementStandardDeviation)}
            };
        }

        @Override
        public double[] residual(double[] measurement, double[] predictedMeasurement) {
            return new double[]{
                    measurement[X] - predictedMeasurement[X],
                    measurement[Y] - predictedMeasurement[Y],
                    Angle.normalizeSigned(measurement[HEADING] - predictedMeasurement[HEADING])
            };
        }
    }

    private static final class Tuning {
        final double initialPositionStandardDeviation;
        final double initialHeadingStandardDeviation;
        final double initialVelocityStandardDeviation;
        final double initialAngularVelocityStandardDeviation;
        final double linearAccelerationStandardDeviation;
        final double angularAccelerationStandardDeviation;
        final double positionMeasurementStandardDeviation;
        final double headingMeasurementStandardDeviation;

        private Tuning(
                double initialPositionStandardDeviation,
                double initialHeadingStandardDeviation,
                double initialVelocityStandardDeviation,
                double initialAngularVelocityStandardDeviation,
                double linearAccelerationStandardDeviation,
                double angularAccelerationStandardDeviation,
                double positionMeasurementStandardDeviation,
                double headingMeasurementStandardDeviation
        ) {
            this.initialPositionStandardDeviation = requirePositiveFinite(
                    "initialPositionStandardDeviation", initialPositionStandardDeviation
            );
            this.initialHeadingStandardDeviation = requirePositiveFinite(
                    "initialHeadingStandardDeviation", initialHeadingStandardDeviation
            );
            this.initialVelocityStandardDeviation = requirePositiveFinite(
                    "initialVelocityStandardDeviation", initialVelocityStandardDeviation
            );
            this.initialAngularVelocityStandardDeviation = requirePositiveFinite(
                    "initialAngularVelocityStandardDeviation", initialAngularVelocityStandardDeviation
            );
            this.linearAccelerationStandardDeviation = requirePositiveFinite(
                    "linearAccelerationStandardDeviation", linearAccelerationStandardDeviation
            );
            this.angularAccelerationStandardDeviation = requirePositiveFinite(
                    "angularAccelerationStandardDeviation", angularAccelerationStandardDeviation
            );
            this.positionMeasurementStandardDeviation = requirePositiveFinite(
                    "positionMeasurementStandardDeviation", positionMeasurementStandardDeviation
            );
            this.headingMeasurementStandardDeviation = requirePositiveFinite(
                    "headingMeasurementStandardDeviation", headingMeasurementStandardDeviation
            );
        }

        static Tuning from(EKFPinpointConfig config) {
            return new Tuning(
                    config.initialPositionStandardDeviation.get(),
                    config.initialHeadingStandardDeviation.get(),
                    config.initialVelocityStandardDeviation.get(),
                    config.initialAngularVelocityStandardDeviation.get(),
                    config.linearAccelerationStandardDeviation.get(),
                    config.angularAccelerationStandardDeviation.get(),
                    config.positionMeasurementStandardDeviation.get(),
                    config.headingMeasurementStandardDeviation.get()
            );
        }

        private static double requirePositiveFinite(String name, double value) {
            if (!Double.isFinite(value) || value <= 0.0) {
                throw new IllegalArgumentException(name + " must be finite and positive.");
            }
            return value;
        }
    }
}
