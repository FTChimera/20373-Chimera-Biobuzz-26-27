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

    public long get() {
        return unit.convert(System.nanoTime() - startTime, TimeUnit.NANOSECONDS);
    }
    public long get(TimeUnit timeUnit) {
        return timeUnit.convert(System.nanoTime() - startTime, TimeUnit.NANOSECONDS);
    }

    public boolean past(long time) {
        return get() > time;
    }
    public boolean past(long time, TimeUnit timeUnit) {
        return get(timeUnit) > time;
    }
    public long dt() {
        long now = System.nanoTime();
        long dt = unit.convert(now - dtPrev, TimeUnit.NANOSECONDS);
        dtPrev = now;
        return dt;
    }
    public long dt(TimeUnit timeUnit) {
        long now = System.nanoTime();
        long dt = timeUnit.convert(now - dtPrev, TimeUnit.NANOSECONDS);
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

}
