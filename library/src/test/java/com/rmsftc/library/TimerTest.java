package com.rmsftc.library;

import org.junit.Test;

import java.util.concurrent.TimeUnit;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

public class TimerTest {
    @Test
    public void reportsElapsedTimeDtAndResetsInConfiguredUnits() {
        Timer timer = new Timer(TimeUnit.MILLISECONDS);
        double now = System.nanoTime();
        timer.startTime = (long) (now - TimeUnit.MILLISECONDS.toNanos(25));
        timer.dtPrev = (long) (now - TimeUnit.MILLISECONDS.toNanos(15));

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
}
