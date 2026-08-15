package com.rmsftc.library.filters;

public interface NoiseFilter {
    void updateState(double newState);
    void resetState();
    double getState();
}
