package org.firstinspires.ftc.teamcode.library;

import org.junit.Test;

import java.util.concurrent.TimeUnit;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

public class TimerTest {
    @Test
    public void reportsElapsedTimeDtAndResetsInConfiguredUnits() {
        Timer timer = new Timer(TimeUnit.MILLISECONDS);
        long now = System.nanoTime();
        timer.startTime = now - TimeUnit.MILLISECONDS.toNanos(25);
        timer.dtPrev = now - TimeUnit.MILLISECONDS.toNanos(15);

        assertTrue(timer.get() >= 20);
        assertTrue(timer.get(TimeUnit.MILLISECONDS) >= 20);
        assertTrue(timer.past(10));
        assertTrue(timer.past(10, TimeUnit.MILLISECONDS));
        assertTrue(timer.dt() >= 10);

        timer.setUnit(TimeUnit.SECONDS);
        assertEquals(TimeUnit.SECONDS, timer.getUnit());

        timer.reset();
        assertTrue(timer.get(TimeUnit.MILLISECONDS) < 50);
        assertTrue(timer.dt(TimeUnit.MILLISECONDS) < 50);
    }

    @Test
    public void convertsElapsedTimeUsingTimeUnitRatios() {
        Timer timer = new Timer(TimeUnit.NANOSECONDS);
        long now = System.nanoTime();
        timer.startTime = now - TimeUnit.HOURS.toNanos(48);
        timer.dtPrev = now - TimeUnit.SECONDS.toNanos(90);

        assertEquals(48.0, timer.get(TimeUnit.HOURS), 0.05);
        assertEquals(2.0, timer.get(TimeUnit.DAYS), 0.01);
        assertEquals(2880.0, timer.get(TimeUnit.MINUTES), 1.0);
        assertEquals(90.0, timer.dt(TimeUnit.SECONDS), 0.1);
    }
}
