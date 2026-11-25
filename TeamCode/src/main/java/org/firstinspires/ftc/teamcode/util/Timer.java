package org.firstinspires.ftc.teamcode.util;

import com.rmsftc.utils.Clock;
import java.util.Objects;
import java.util.concurrent.TimeUnit;

public class Timer {
    private final Clock clock;
    long startTime; // nanoseconds
    long dtPrev; // nanoseconds
    public TimeUnit unit;
    public Timer() {
        this(TimeUnit.MILLISECONDS);
    }
    /** A utility timer with nanosecond accuracy that can also calculate dt */
    public Timer(TimeUnit unit) {
        this(unit, Clock.SYSTEM);
    }

    public Timer(TimeUnit unit, Clock clock) {
        this.clock = Objects.requireNonNull(clock, "clock");
        startTime = clock.nowNanos();
        dtPrev = startTime;
        this.unit = unit;
    }

    public double get() {
        return convertFromNanoseconds(clock.nowNanos() - startTime, unit);
    }
    public double get(TimeUnit timeUnit) {
        return convertFromNanoseconds(clock.nowNanos() - startTime, timeUnit);
    }

    public boolean past(double time) {
        return get() > time;
    }
    public boolean past(double time, TimeUnit timeUnit) {
        return get(timeUnit) > time;
    }
    public double dt() {
        long now = clock.nowNanos();
        double dt = convertFromNanoseconds(now - dtPrev, unit);
        dtPrev = now;
        return dt;
    }
    public double dt(TimeUnit timeUnit) {
        long now = clock.nowNanos();
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
        startTime = clock.nowNanos();
        dtPrev = startTime;
    }

    private double convertFromNanoseconds(double nanoseconds, TimeUnit timeUnit) {
        return convert(nanoseconds, TimeUnit.NANOSECONDS, timeUnit);
    }

    private double convert(double value, TimeUnit fromUnit, TimeUnit toUnit) {
        return value * fromUnit.toNanos(1) / toUnit.toNanos(1);
    }

}
