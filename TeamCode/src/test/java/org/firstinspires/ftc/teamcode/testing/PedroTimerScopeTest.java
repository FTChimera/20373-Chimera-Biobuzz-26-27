package org.firstinspires.ftc.teamcode.testing;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import com.pedropathing.utils.Timer;

import org.firstinspires.ftc.teamcode.util.FakeClock;
import org.junit.Test;
import org.mockito.Mockito;

import java.util.concurrent.TimeUnit;

public class PedroTimerScopeTest {
    @Test
    public void vendorTimersUseSimulationTimeAndConstructionIsRestored() {
        FakeClock clock = new FakeClock();
        try (PedroTimerScope ignored = new PedroTimerScope(clock)) {
            Timer timer = new Timer();
            assertTrue(Mockito.mockingDetails(timer).isMock());
            clock.add(25_500_000);
            assertEquals(25_500_000, timer.nanoseconds());
            assertEquals(25, timer.get(TimeUnit.MILLISECONDS), 0);
            assertEquals(0.0255, timer.seconds(), 1e-12);
            timer.reset();
            assertEquals(0, timer.nanoseconds());
        }
        assertFalse(Mockito.mockingDetails(new Timer()).isMock());
    }
}
