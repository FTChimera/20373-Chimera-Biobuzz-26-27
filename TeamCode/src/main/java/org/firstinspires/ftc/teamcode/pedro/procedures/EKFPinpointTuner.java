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
        double linearAccelerationStandardDeviation = accelerationStandardDeviation(
                average(forward.linearAccelerationMeanSquare(), strafe.linearAccelerationMeanSquare()),
                stationary.linearAccelerationMeanSquare(),
                MIN_LINEAR_ACCELERATION_STANDARD_DEVIATION
        );
        double angularAccelerationStandardDeviation = accelerationStandardDeviation(
                turn.angularAccelerationMeanSquare(),
                stationary.angularAccelerationMeanSquare(),
                MIN_ANGULAR_ACCELERATION_STANDARD_DEVIATION
        );

        result("positionMeasurementStandardDeviation", positionMeasurementStandardDeviation);
        result("headingMeasurementStandardDeviation", headingMeasurementStandardDeviation);
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
}
