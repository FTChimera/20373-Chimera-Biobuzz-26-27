package org.firstinspires.ftc.teamcode.util;

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
        return convert(nanoseconds, TimeUnit.NANOSECONDS, timeUnit);
    }

    private double convert(double value, TimeUnit fromUnit, TimeUnit toUnit) {
        return value * fromUnit.toNanos(1) / toUnit.toNanos(1);
    }

}
