package org.firstinspires.ftc.teamcode.util;

import com.rmsftc.utils.Clock;
import com.rmsftc.utils.math.units.NumericalValue;
import com.rmsftc.utils.math.units.basic.TimeUnit;

import java.util.concurrent.atomic.AtomicLong;

/** Controllable {@link Clock} for tests: time only moves when the test moves it. */
public final class FakeClock implements Clock {
    private final AtomicLong nowNanos = new AtomicLong();

    public void set(long nanos) {
        nowNanos.set(nanos);
    }

    public void add(long nanos) {
        nowNanos.addAndGet(nanos);
    }

    @Override public NumericalValue<TimeUnit> get() {
        return NumericalValue.of(nowNanos.get(), TimeUnit.NANOSECOND);
    }

    @Override public long nowNanos() {
        return nowNanos.get();
    }
}
