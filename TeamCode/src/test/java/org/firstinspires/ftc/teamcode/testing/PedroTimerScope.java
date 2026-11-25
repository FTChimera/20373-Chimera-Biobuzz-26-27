package org.firstinspires.ftc.teamcode.testing;

import static org.mockito.ArgumentMatchers.any;
import static org.mockito.Mockito.doAnswer;

import com.pedropathing.utils.Timer;
import com.rmsftc.utils.Clock;

import org.mockito.MockedConstruction;
import org.mockito.Mockito;

import java.util.Objects;
import java.util.concurrent.TimeUnit;

/** Adapts the vendor's non-injectable timer without replacing its path/control algorithms. */
public final class PedroTimerScope implements AutoCloseable {
    private final MockedConstruction<Timer> timers;

    public PedroTimerScope(Clock clock) {
        Objects.requireNonNull(clock, "clock");
        timers = Mockito.mockConstruction(Timer.class,
                Mockito.withSettings().defaultAnswer(invocation -> {
                    throw new UnsupportedOperationException("Unsupported Pedro timer call: "
                            + invocation.getMethod());
                }),
                (timer, context) -> {
                    long[] start = {clock.nowNanos()};
                    doAnswer(invocation -> {
                        start[0] = clock.nowNanos();
                        return null;
                    }).when(timer).reset();
                    doAnswer(invocation -> clock.nowNanos() - start[0]).when(timer).nanoseconds();
                    doAnswer(invocation -> {
                        TimeUnit unit = invocation.getArgument(0);
                        return (double) unit.convert(clock.nowNanos() - start[0], TimeUnit.NANOSECONDS);
                    }).when(timer).get(any(TimeUnit.class));
                    doAnswer(invocation -> (double) TimeUnit.MILLISECONDS.convert(
                            clock.nowNanos() - start[0], TimeUnit.NANOSECONDS)).when(timer).milliseconds();
                    doAnswer(invocation -> (clock.nowNanos() - start[0]) / 1e9).when(timer).seconds();
                });
    }

    @Override public void close() { timers.close(); }
}
