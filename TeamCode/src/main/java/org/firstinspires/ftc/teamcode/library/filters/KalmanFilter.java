package org.firstinspires.ftc.teamcode.library.filters;

public class KalmanFilter implements NoiseFilter {
    private final double processNoise;
    private final double measurementNoise;
    private final double initialEstimateError;
    private double estimateError;
    private double kalmanGain;
    private double state;
    private boolean hasEstimate;

    public KalmanFilter() {
        this(1e-3, 1e-2, 1.0);
    }

    public KalmanFilter(double processNoise, double measurementNoise) {
        this(processNoise, measurementNoise, 1.0);
    }

    public KalmanFilter(double processNoise, double measurementNoise, double initialEstimateError) {
        if (processNoise < 0) {
            throw new IllegalArgumentException("processNoise must be non-negative.");
        }
        if (measurementNoise <= 0) {
            throw new IllegalArgumentException("measurementNoise must be positive.");
        }
        if (initialEstimateError <= 0) {
            throw new IllegalArgumentException("initialEstimateError must be positive.");
        }
        this.processNoise = processNoise;
        this.measurementNoise = measurementNoise;
        this.initialEstimateError = initialEstimateError;
        resetState();
    }

    public boolean hasEstimate() {
        return hasEstimate;
    }

    public double getEstimateError() {
        return estimateError;
    }

    public double getKalmanGain() {
        return kalmanGain;
    }

    @Override
    public void updateState(double newState) {
        if (!hasEstimate) {
            state = newState;
            kalmanGain = 1.0;
            hasEstimate = true;
            return;
        }

        estimateError += processNoise;
        kalmanGain = estimateError / (estimateError + measurementNoise);
        state += kalmanGain * (newState - state);
        estimateError *= 1.0 - kalmanGain;
    }

    @Override
    public void resetState() {
        state = 0.0;
        estimateError = initialEstimateError;
        kalmanGain = 0.0;
        hasEstimate = false;
    }

    @Override
    public double getState() {
        return state;
    }
}
