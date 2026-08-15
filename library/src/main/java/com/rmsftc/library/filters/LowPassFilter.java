package com.rmsftc.library.filters;

public class LowPassFilter implements NoiseFilter {
    boolean firstUpdate;
    public final double alpha;
    double lastState;


    public LowPassFilter() {
        this(0.3);
    }

    public LowPassFilter(double alpha) {
        if (alpha < 0 || alpha > 1) {
            throw new IllegalArgumentException("Alpha must be between 0 and 1 inclusive");
        }
        this.alpha = alpha;
    }

    @Override
    public void updateState(double newState) {
        if (!firstUpdate) {
            lastState = newState;
            firstUpdate = true;
            return;
        }
        lastState = lastState * (1 - alpha) + newState * alpha;
    }

    @Override
    public void resetState() {
        lastState = 0;
        firstUpdate = false;
    }

    @Override
    public double getState() {
        return lastState;
    }
}