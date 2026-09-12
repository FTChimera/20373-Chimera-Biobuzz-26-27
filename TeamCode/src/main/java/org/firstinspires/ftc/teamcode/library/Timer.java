package org.firstinspires.ftc.teamcode.library;

import java.util.concurrent.TimeUnit;

public class Timer {
    long startTime; // nanoseconds
    long dtPrev; // nanoseconds
    public TimeUnit unit;
    public Timer() {
        this(TimeUnit.MILLISECONDS);
    }
    /** A utility timer with nanosecond accuracy that can also calculate dt */
    public Timer(TimeUnit unit) {
        startTime = System.nanoTime();
        dtPrev = startTime;
        this.unit = unit;
    }

    public double get() {
        return convertFromNanoseconds(System.nanoTime() - startTime, unit);
    }
    public double get(TimeUnit timeUnit) {
        return convertFromNanoseconds(System.nanoTime() - startTime, timeUnit);
    }

    public boolean past(double time) {
        return get() > time;
    }
    public boolean past(double time, TimeUnit timeUnit) {
        return get(timeUnit) > time;
    }
    public double dt() {
        long now = System.nanoTime();
        double dt = convertFromNanoseconds(now - dtPrev, unit);
        dtPrev = now;
        return dt;
    }
    public double dt(TimeUnit timeUnit) {
        long now = System.nanoTime();
        double dt = convertFromNanoseconds(now - dtPrev, timeUnit);
        dtPrev = now;
        return dt;
    }
    public void setUnit(TimeUnit unit) {
        this.unit = unit;
    }
    public TimeUnit getUnit() {
        return this.unit;
    }
    public void reset() {
        startTime = System.nanoTime();
        dtPrev = startTime;
    }

    private double convertFromNanoseconds(double nanoseconds, TimeUnit timeUnit) {
        switch (timeUnit) {
            // Replace with TimeUnit scale (if you cmd click and scroll u will see)
            case NANOSECONDS:
                return nanoseconds;
            case MICROSECONDS:
                return nanoseconds / 1_000.0;
            case MILLISECONDS:
                return nanoseconds / 1_000_000.0;
            case SECONDS:
                return nanoseconds / 1_000_000_000.0;
            case MINUTES:
                return nanoseconds / (60.0 * 1_000_000_000.0);
            case HOURS:
                return nanoseconds / (60.0 * 60.0 * 1_000_000_000.0);
            case DAYS:
                return nanoseconds / (24.0 * 60.0 * 60.0 * 1_000_000_000.0);
            default:
                throw new IllegalArgumentException("Unsupported TimeUnit: " + timeUnit);
        }
    }

}
