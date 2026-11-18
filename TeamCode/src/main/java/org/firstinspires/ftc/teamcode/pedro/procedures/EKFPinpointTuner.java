package org.firstinspires.ftc.teamcode.pedro.procedures;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.localization.Localizer;
import com.pedropathing.localization.MotionState;
import com.pedropathing.math.Pose;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.pedropathing.tuning.autotune.Inputs;
import com.pedropathing.tuning.autotune.Procedure;
import com.pedropathing.tuning.autotune.TuningOpMode;
import com.pedropathing.utils.Angle;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedro.Constants;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

/**
 * Tunes the observable EKF noise terms from Pinpoint's repeatability and drivetrain motion.
 * The robot must have an already-calibrated Pinpoint and a clear path for the driven stages.
 */
public class EKFPinpointTuner extends Procedure {
    private static final double MIN_POSITION_STANDARD_DEVIATION = 0.1;
    private static final double MIN_HEADING_STANDARD_DEVIATION = Math.toRadians(1.0);
    private static final double MIN_LINEAR_ACCELERATION_STANDARD_DEVIATION = 5.0;
    private static final double MIN_ANGULAR_ACCELERATION_STANDARD_DEVIATION = 0.25;
    // The estimator must not ring faster than this. Above ~1 Hz the estimator's
    // wobble is fast enough for the follower to turn it into physical swaying;
    // below it the robot's inertia filters it out.
    private static final double MAX_ESTIMATOR_RING_FREQUENCY_HZ = 1.0;
    // Loop period the ring-frequency cap is computed at. The cap is nearly
    // insensitive to this (it moves by under 1% from 10 ms to 50 ms), so a
    // nominal 20 ms loop is fine.
    private static final double NOMINAL_LOOP_DT_SECONDS = 0.02;

    public EKFPinpointTuner() {
        super(
                "EKF Pinpoint Tuner",
                "Measures stationary Pinpoint repeatability, then drives forward, strafes, and turns "
                        + "to estimate EKF measurement and process noise. Run Pinpoint Tuner first."
        );
    }

    @Override
    public void run() throws InterruptedException {
        Inputs setup = inputs(
                "EKF Pinpoint Tuner Setup",
                "Place the robot on open field tiles. The robot will drive forward, strafe left, "
                        + "and turn counterclockwise without user control."
        );
        Inputs.Field<Double> stationarySeconds = setup.d("Stationary sample duration (seconds)")
                .min(2.0).max(15.0).withDefault(5.0);
        Inputs.Field<Double> travelDistance = setup.d("Forward and strafe distance (inches)")
                .min(12.0).max(72.0).withDefault(36.0);
        Inputs.Field<Double> drivePower = setup.d("Forward and strafe power")
                .min(0.15).max(0.6).withDefault(0.35);
        Inputs.Field<Double> turnDegrees = setup.d("Turn angle (degrees)")
                .min(90.0).max(720.0).withDefault(360.0);
        Inputs.Field<Double> turnPower = setup.d("Turn power")
                .min(0.15).max(0.5).withDefault(0.3);
        awaitInputs(setup);

        confirmation(
                "Motion Safety",
                "Clear the requested path and keep the robot away from walls. Each driven stage stops "
                        + "after its target distance or an eight-second safety timeout."
        );

        EKFPinpointStationaryNoise stationary = runOpMode(
                new EKFPinpointStationaryNoise(stationarySeconds.get())
        );
        EKFPinpointMotionNoise forward = runOpMode(
                EKFPinpointMotionNoise.forward(travelDistance.get(), drivePower.get())
        );
        EKFPinpointMotionNoise strafe = runOpMode(
                EKFPinpointMotionNoise.strafe(travelDistance.get(), drivePower.get())
        );
        EKFPinpointMotionNoise turn = runOpMode(
                EKFPinpointMotionNoise.turn(Math.toRadians(turnDegrees.get()), turnPower.get())
        );

        if (!stationary.hasEnoughSamples()) {
            abort("Stationary sampling produced too few readings. Keep the robot still and run the tuner again.");
            return;
        }
        if (!forward.hasReachedTarget() || !strafe.hasReachedTarget() || !turn.hasReachedTarget()) {
            abort("A driven stage timed out before reaching its target. Check the clear path, Pinpoint, and drivetrain, then run again.");
            return;
        }
        if (!forward.hasEnoughSamples() || !strafe.hasEnoughSamples() || !turn.hasEnoughSamples()) {
            abort("A driven stage produced too few readings. Check Pinpoint and drivetrain operation, then run again.");
            return;
        }

        double positionMeasurementStandardDeviation = Math.max(
                MIN_POSITION_STANDARD_DEVIATION,
                Math.sqrt(stationary.positionVariance())
        );
        double headingMeasurementStandardDeviation = Math.max(
                MIN_HEADING_STANDARD_DEVIATION,
                Math.sqrt(stationary.headingVariance())
        );
        double measuredLinearAccelerationStandardDeviation = accelerationStandardDeviation(
                average(forward.linearAccelerationMeanSquare(), strafe.linearAccelerationMeanSquare()),
                stationary.linearAccelerationMeanSquare(),
                MIN_LINEAR_ACCELERATION_STANDARD_DEVIATION
        );
        double measuredAngularAccelerationStandardDeviation = accelerationStandardDeviation(
                turn.angularAccelerationMeanSquare(),
                stationary.angularAccelerationMeanSquare(),
                MIN_ANGULAR_ACCELERATION_STANDARD_DEVIATION
        );
        // The measured RMS acceleration is dominated by the spin-up/spin-down
        // transients of the driven stages, so it overestimates the process noise
        // for normal driving. An oversized process noise makes the estimator
        // underdamped: it rings at a few Hz, and the follower turns that wobble
        // into physical swaying (the heading estimator rang at ~2.6 Hz with the
        // measured 9.08 rad/s^2, and the position estimator at ~3.8 Hz with the
        // measured 114.46 in/s^2). Cap both process-noise terms so the
        // estimator's ring frequency stays below MAX_ESTIMATOR_RING_FREQUENCY_HZ.
        // The cap is computed from the measured measurement noise, so it adapts;
        // the measured values are reported alongside the capped ones. If the
        // robot lags on fast moves after this, raise the capped value toward
        // the measured one.
        double linearAccelerationStandardDeviation = Math.min(
                measuredLinearAccelerationStandardDeviation,
                EKFPinpointTuningMath.maxAccelerationStandardDeviation(
                        positionMeasurementStandardDeviation,
                        NOMINAL_LOOP_DT_SECONDS,
                        MAX_ESTIMATOR_RING_FREQUENCY_HZ)
        );
        double angularAccelerationStandardDeviation = Math.min(
                measuredAngularAccelerationStandardDeviation,
                EKFPinpointTuningMath.maxAccelerationStandardDeviation(
                        headingMeasurementStandardDeviation,
                        NOMINAL_LOOP_DT_SECONDS,
                        MAX_ESTIMATOR_RING_FREQUENCY_HZ)
        );

        result("positionMeasurementStandardDeviation", positionMeasurementStandardDeviation);
        result("headingMeasurementStandardDeviation", headingMeasurementStandardDeviation);
        result("measuredLinearAccelerationStandardDeviation", measuredLinearAccelerationStandardDeviation);
        result("measuredAngularAccelerationStandardDeviation", measuredAngularAccelerationStandardDeviation);
        result("linearAccelerationStandardDeviation", linearAccelerationStandardDeviation);
        result("angularAccelerationStandardDeviation", angularAccelerationStandardDeviation);
        result("stationarySamples", stationary.sampleCount());
        result("forwardAccelerationSamples", forward.accelerationSampleCount());
        result("strafeAccelerationSamples", strafe.accelerationSampleCount());
        result("turnAccelerationSamples", turn.accelerationSampleCount());

        code(Procedure.Language.JAVA, configCode(
                positionMeasurementStandardDeviation,
                headingMeasurementStandardDeviation,
                linearAccelerationStandardDeviation,
                angularAccelerationStandardDeviation
        ));
    }

    private static double accelerationStandardDeviation(
            double motionMeanSquare,
            double stationaryMeanSquare,
            double minimum
    ) {
        return Math.max(minimum, Math.sqrt(Math.max(0.0, motionMeanSquare - stationaryMeanSquare)));
    }

    private static double average(double first, double second) {
        return (first + second) / 2.0;
    }

    private static String configCode(
            double positionMeasurementStandardDeviation,
            double headingMeasurementStandardDeviation,
            double linearAccelerationStandardDeviation,
            double angularAccelerationStandardDeviation
    ) {
        return "// Paste these assignments into the existing Constants.PINPOINT_CONFIG lambda.\n"
                + "c.initialPositionStandardDeviation.set("
                + format(Constants.PINPOINT_CONFIG.initialPositionStandardDeviation.get()) + ");\n"
                + "c.initialHeadingStandardDeviation.set("
                + format(Constants.PINPOINT_CONFIG.initialHeadingStandardDeviation.get()) + ");\n"
                + "c.initialVelocityStandardDeviation.set("
                + format(Constants.PINPOINT_CONFIG.initialVelocityStandardDeviation.get()) + ");\n"
                + "c.initialAngularVelocityStandardDeviation.set("
                + format(Constants.PINPOINT_CONFIG.initialAngularVelocityStandardDeviation.get()) + ");\n"
                + "c.linearAccelerationStandardDeviation.set("
                + format(linearAccelerationStandardDeviation) + ");\n"
                + "c.angularAccelerationStandardDeviation.set("
                + format(angularAccelerationStandardDeviation) + ");\n"
                + "c.positionMeasurementStandardDeviation.set("
                + format(positionMeasurementStandardDeviation) + ");\n"
                + "c.headingMeasurementStandardDeviation.set("
                + format(headingMeasurementStandardDeviation) + ");";
    }

    private static String format(double value) {
        return String.format(Locale.US, "%.8f", value);
    }
}

class EKFPinpointStationaryNoise extends TuningOpMode<EKFPinpointStationaryNoise> {
    private final double durationSeconds;
    private final EKFPinpointSampleCollector samples = new EKFPinpointSampleCollector();

    EKFPinpointStationaryNoise(double durationSeconds) {
        super(
                "Stationary Pinpoint Noise",
                "Keep the robot completely still while Pinpoint repeatability is sampled for "
                        + durationSeconds + " seconds.",
                false
        );
        this.durationSeconds = durationSeconds;
    }

    @Override
    protected EKFPinpointStationaryNoise runTuningOpMode() {
        Localizer localizer = new PinpointLocalizer(hardwareMap, Constants.PINPOINT_CONFIG);
        localizer.setPose(Pose.zero());
        localizer.update();

        waitForStart();
        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive() && timer.seconds() < durationSeconds) {
            localizer.update();
            samples.add(localizer.state(), System.nanoTime());
            telemetry.addData("Samples", samples.sampleCount());
            telemetry.addData("Remaining seconds", Math.max(0.0, durationSeconds - timer.seconds()));
            telemetry.update();
            idle();
        }
        return this;
    }

    boolean hasEnoughSamples() {
        return samples.sampleCount() >= 20 && samples.accelerationSampleCount() >= 10;
    }

    int sampleCount() {
        return samples.sampleCount();
    }

    double positionVariance() {
        return (samples.xVariance() + samples.yVariance()) / 2.0;
    }

    double headingVariance() {
        return samples.headingVariance();
    }

    double linearAccelerationMeanSquare() {
        return samples.linearAccelerationMeanSquare();
    }

    double angularAccelerationMeanSquare() {
        return samples.angularAccelerationMeanSquare();
    }
}

class EKFPinpointMotionNoise extends TuningOpMode<EKFPinpointMotionNoise> {
    private static final double MAX_STAGE_SECONDS = 8.0;
    private final Motion motion;
    private final double target;
    private final double power;
    private final EKFPinpointSampleCollector samples = new EKFPinpointSampleCollector();
    private boolean reachedTarget;

    private EKFPinpointMotionNoise(Motion motion, double target, double power) {
        super(motion.title, motion.description(target, power), false);
        this.motion = motion;
        this.target = target;
        this.power = power;
    }

    static EKFPinpointMotionNoise forward(double distance, double power) {
        return new EKFPinpointMotionNoise(Motion.FORWARD, distance, power);
    }

    static EKFPinpointMotionNoise strafe(double distance, double power) {
        return new EKFPinpointMotionNoise(Motion.STRAFE, distance, power);
    }

    static EKFPinpointMotionNoise turn(double angleRadians, double power) {
        return new EKFPinpointMotionNoise(Motion.TURN, angleRadians, power);
    }

    @Override
    protected EKFPinpointMotionNoise runTuningOpMode() {
        Localizer localizer = new PinpointLocalizer(hardwareMap, Constants.PINPOINT_CONFIG);
        Mecanum drivetrain = new Mecanum(hardwareMap, Constants.MECANUM_CONFIG);
        localizer.setPose(Pose.zero());
        localizer.update();

        waitForStart();
        ElapsedTime timer = new ElapsedTime();
        double accumulatedHeading = 0.0;
        double previousHeading = localizer.pose().heading();

        try {
            while (opModeIsActive() && timer.seconds() < MAX_STAGE_SECONDS) {
                localizer.update();
                MotionState state = localizer.state();
                samples.add(state, System.nanoTime());

                if (motion == Motion.TURN) {
                    accumulatedHeading += Angle.normalizeSigned(state.pose().heading() - previousHeading);
                    previousHeading = state.pose().heading();
                    if (Math.abs(accumulatedHeading) >= target) {
                        reachedTarget = true;
                        break;
                    }
                } else if (Math.hypot(state.pose().x(), state.pose().y()) >= target) {
                    reachedTarget = true;
                    break;
                }

                drivetrain.drive(motion.powers(power), false);
                telemetry.addData("Motion", motion.title);
                telemetry.addData("Target", motion == Motion.TURN ? Math.toDegrees(target) : target);
                telemetry.addData("Progress", motion == Motion.TURN
                        ? Math.toDegrees(Math.abs(accumulatedHeading))
                        : Math.hypot(state.pose().x(), state.pose().y()));
                telemetry.addData("Acceleration samples", samples.accelerationSampleCount());
                telemetry.update();
                idle();
            }
        } finally {
            drivetrain.stop(true);
        }
        return this;
    }

    boolean hasEnoughSamples() {
        return samples.accelerationSampleCount() >= 20;
    }

    boolean hasReachedTarget() {
        return reachedTarget;
    }

    int accelerationSampleCount() {
        return samples.accelerationSampleCount();
    }

    double linearAccelerationMeanSquare() {
        return samples.linearAccelerationMeanSquare();
    }

    double angularAccelerationMeanSquare() {
        return samples.angularAccelerationMeanSquare();
    }

    private enum Motion {
        FORWARD("Forward Process Noise", "The robot will drive forward %.1f inches at %.2f power."),
        STRAFE("Strafe Process Noise", "The robot will strafe left %.1f inches at %.2f power."),
        TURN("Angular Process Noise", "The robot will turn %.0f degrees counterclockwise at %.2f power.");

        final String title;
        final String descriptionFormat;

        Motion(String title, String descriptionFormat) {
            this.title = title;
            this.descriptionFormat = descriptionFormat;
        }

        String description(double target, double power) {
            double displayTarget = this == TURN ? Math.toDegrees(target) : target;
            return String.format(Locale.US, descriptionFormat, displayTarget, power);
        }

        DrivePowers powers(double power) {
            switch (this) {
                case FORWARD:
                    return new DrivePowers(power, 0.0, 0.0);
                case STRAFE:
                    return new DrivePowers(0.0, power, 0.0);
                case TURN:
                    return new DrivePowers(0.0, 0.0, power);
                default:
                    throw new IllegalStateException("Unsupported motion: " + this);
            }
        }
    }
}

class EKFPinpointSampleCollector {
    private static final double MIN_SAMPLE_INTERVAL_SECONDS = 0.002;
    private static final double MAX_SAMPLE_INTERVAL_SECONDS = 0.1;

    private final List<Double> x = new ArrayList<>();
    private final List<Double> y = new ArrayList<>();
    private final List<Double> headingError = new ArrayList<>();
    private final List<Double> linearAccelerationMeanSquares = new ArrayList<>();
    private final List<Double> angularAccelerationMeanSquares = new ArrayList<>();
    private double initialHeading;
    private boolean hasInitialHeading;
    private MotionState previousState;
    private long previousNanos;

    void add(MotionState state, long nanos) {
        Pose pose = state.pose();
        if (!hasInitialHeading) {
            initialHeading = pose.heading();
            hasInitialHeading = true;
        }

        x.add(pose.x());
        y.add(pose.y());
        headingError.add(Angle.normalizeSigned(pose.heading() - initialHeading));

        if (previousState != null) {
            double dt = (nanos - previousNanos) / 1.0e9;
            if (dt >= MIN_SAMPLE_INTERVAL_SECONDS && dt <= MAX_SAMPLE_INTERVAL_SECONDS) {
                double ax = (state.velocity().vx - previousState.velocity().vx) / dt;
                double ay = (state.velocity().vy - previousState.velocity().vy) / dt;
                double alpha = (state.velocity().omega - previousState.velocity().omega) / dt;
                linearAccelerationMeanSquares.add((ax * ax + ay * ay) / 2.0);
                angularAccelerationMeanSquares.add(alpha * alpha);
            }
        }

        previousState = state;
        previousNanos = nanos;
    }

    int sampleCount() {
        return x.size();
    }

    int accelerationSampleCount() {
        return linearAccelerationMeanSquares.size();
    }

    double xVariance() {
        return EKFPinpointTuningMath.variance(x);
    }

    double yVariance() {
        return EKFPinpointTuningMath.variance(y);
    }

    double headingVariance() {
        return EKFPinpointTuningMath.variance(headingError);
    }

    double linearAccelerationMeanSquare() {
        return EKFPinpointTuningMath.mean(linearAccelerationMeanSquares);
    }

    double angularAccelerationMeanSquare() {
        return EKFPinpointTuningMath.mean(angularAccelerationMeanSquares);
    }
}

final class EKFPinpointTuningMath {
    private EKFPinpointTuningMath() {
    }

    static double mean(List<Double> values) {
        if (values.isEmpty()) {
            return 0.0;
        }

        double sum = 0.0;
        for (double value : values) {
            sum += value;
        }
        return sum / values.size();
    }

    static double variance(List<Double> values) {
        if (values.size() < 2) {
            return 0.0;
        }

        double mean = mean(values);
        double squaredErrorSum = 0.0;
        for (double value : values) {
            double error = value - mean;
            squaredErrorSum += error * error;
        }
        return squaredErrorSum / (values.size() - 1);
    }

    /**
     * Steady-state ring frequency (Hz) of a 2-state constant-velocity
     * (alpha-beta) estimator with the given acceleration process noise and
     * measurement noise, at the given loop period. The heading/omega block
     * and each position/velocity block of the EKF are decoupled 2-state
     * filters with exactly this structure (same F and white-noise-acceleration
     * Q as EKFPinpoint.ConstantVelocityDynamics), so this is exact for both
     * axes. Returns 0.0 when the estimator poles are real (no ringing). A
     * ring frequency above ~1 Hz is visible on the robot: the follower turns
     * the estimator's wobble into oscillating drive power.
     */
    static double ringFrequencyHz(
            double accelerationStandardDeviation,
            double measurementStandardDeviation,
            double dtSeconds) {
        double dt = dtSeconds;
        double dt2 = dt * dt;
        double dt3 = dt2 * dt;
        double dt4 = dt2 * dt2;
        double accelerationVariance = accelerationStandardDeviation * accelerationStandardDeviation;
        double q00 = dt4 * accelerationVariance / 4.0;
        double q01 = dt3 * accelerationVariance / 2.0;
        double q11 = dt2 * accelerationVariance;
        double r = measurementStandardDeviation * measurementStandardDeviation;

        // Iterate the discrete Riccati equation to steady state.
        // F = [[1, dt], [0, 1]], H = [1, 0].
        double p00 = 1.0;
        double p01 = 0.0;
        double p11 = 1.0;
        double k0 = 0.0;
        double k1 = 0.0;
        for (int i = 0; i < 500; i++) {
            double pp00 = p00 + 2.0 * dt * p01 + dt2 * p11 + q00;
            double pp01 = p01 + dt * p11 + q01;
            double pp11 = p11 + q11;
            double s = pp00 + r;
            k0 = pp00 / s;
            k1 = pp01 / s;
            p00 = pp00 - k0 * pp00;
            p01 = pp01 - k0 * pp01;
            p11 = pp11 - k1 * pp01;
        }

        // Closed-loop poles of A = (I - K H) F.
        double a00 = 1.0 - k0;
        double a01 = (1.0 - k0) * dt;
        double a10 = -k1;
        double a11 = 1.0 - k1 * dt;
        double trace = a00 + a11;
        double det = a00 * a11 - a01 * a10;
        double discriminant = trace * trace / 4.0 - det;
        if (discriminant >= 0.0) {
            return 0.0;
        }
        double real = trace / 2.0;
        double imag = Math.sqrt(-discriminant);
        return Math.abs(Math.atan2(imag, real)) / (2.0 * Math.PI * dt);
    }

    /**
     * Largest acceleration standard deviation that keeps the estimator's ring
     * frequency at or below {@code maxRingFrequencyHz}. The ring frequency
     * grows monotonically with the process noise, so this binary-searches for
     * the boundary.
     */
    static double maxAccelerationStandardDeviation(
            double measurementStandardDeviation,
            double dtSeconds,
            double maxRingFrequencyHz) {
        double low = 0.0;
        double high = 1.0;
        while (ringFrequencyHz(high, measurementStandardDeviation, dtSeconds) <= maxRingFrequencyHz) {
            high *= 2.0;
            if (high > 1.0e6) {
                return high;
            }
        }
        for (int i = 0; i < 60; i++) {
            double mid = (low + high) / 2.0;
            if (ringFrequencyHz(mid, measurementStandardDeviation, dtSeconds) <= maxRingFrequencyHz) {
                low = mid;
            } else {
                high = mid;
            }
        }
        return low;
    }
}
