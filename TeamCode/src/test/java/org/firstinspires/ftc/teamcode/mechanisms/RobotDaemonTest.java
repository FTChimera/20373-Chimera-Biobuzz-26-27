package org.firstinspires.ftc.teamcode.mechanisms;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import com.rmsftc.utils.ftc.daemons.FTCDaemon;

import org.junit.Test;

import java.lang.reflect.Method;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;

public class RobotDaemonTest {
    @Test
    public void daemonSignaturesMatchPinnedScannerAndHaveUniquePriorities() {
        List<Integer> priorities = new ArrayList<>();
        for (Method method : Robot.class.getDeclaredMethods()) {
            FTCDaemon annotation = method.getAnnotation(FTCDaemon.class);
            if (annotation == null) continue;
            // FTCUtils 0.1.2's scanner accepts primitive void as well as
            // boxed Void (0.1.1 required the box); either is a valid daemon.
            assertTrue("daemon " + method.getName() + " must return Void or void",
                    Void.class.equals(method.getGenericReturnType())
                            || Void.TYPE.equals(method.getGenericReturnType()));
            int priority = Robot.daemonPriority(annotation.name());
            assertTrue(priority >= 0);
            priorities.add(priority);
        }
        Collections.sort(priorities);
        assertEquals(11, priorities.size());
        for (int i = 0; i < priorities.size(); i++) {
            assertEquals(i, priorities.get(i).intValue());
        }
    }

    @Test
    public void orderDoesNotDependOnScannerRegistrationOrder() {
        List<String> names = new ArrayList<>();
        names.add("robot-mechanisms");
        names.add("robot-follower");
        names.add("robot-clock");
        names.add("external-clock");
        names.sort(Comparator.comparingInt(Robot::daemonPriority));
        assertEquals("external-clock", names.get(0));
        assertEquals("robot-clock", names.get(1));
        assertEquals("robot-follower", names.get(2));
        assertEquals("robot-mechanisms", names.get(3));
    }
}
