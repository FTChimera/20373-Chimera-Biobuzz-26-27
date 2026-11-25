package org.firstinspires.ftc.teamcode.testing;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertThrows;
import static org.junit.Assert.assertTrue;

import android.util.Log;

import org.junit.Test;

public class AndroidLogScopeTest {
    @Test
    public void recordsSdkTransportAndRestoresAndroidBoundary() {
        try (AndroidLogScope logs = new AndroidLogScope()) {
            assertEquals(7, Log.println(Log.INFO, "FTC", "message"));
            Log.e("FTC", "failure", new IllegalStateException("device disconnected"));
            assertTrue(Log.isLoggable("FTC", Log.DEBUG));
            assertEquals(2, logs.entries().size());
            assertEquals(Log.INFO, logs.entries().get(0).priority);
            assertEquals("FTC", logs.entries().get(0).tag);
            assertTrue(logs.entries().get(1).message.contains("device disconnected"));
            assertThrows(UnsupportedOperationException.class, () -> logs.entries().clear());
        }
        assertThrows(RuntimeException.class, () -> Log.println(Log.INFO, "FTC", "not scoped"));
    }
}
