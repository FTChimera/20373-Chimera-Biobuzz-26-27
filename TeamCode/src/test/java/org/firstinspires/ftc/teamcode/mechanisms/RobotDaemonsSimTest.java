package org.firstinspires.ftc.teamcode.mechanisms;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

import com.rmsftc.utils.ftc.DaemonManager;

import org.firstinspires.ftc.teamcode.util.SimEnvironment;
import org.junit.After;
import org.junit.AfterClass;
import org.junit.Before;
import org.junit.Test;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.stream.Collectors;

/**
 * Regression test for the ftcsim robot-clock daemon crash: on a desktop JVM
 * the Sloth classpath scan that registers {@code @FTCDaemon} methods never
 * runs, so the global {@link DaemonManager} registry is empty and
 * {@link Robot#start()} throws {@code IllegalStateException: Expected one
 * robot-clock daemon; found 0} — at START, after INIT passes.
 *
 * <p>The seams tested here are exactly the code {@code start()} runs for its
 * daemon lookup: {@link Robot#ensureDaemonsRegistered()} (the sim-only
 * explicit scan) and {@link Robot#orderedRobotDaemons()} (the registry
 * lookup that throws when a daemon is missing). A full {@code new Robot(...)}
 * plus {@code start()} is deliberately NOT constructed here: it needs drive
 * motors plus a Pinpoint (concrete, hardware-touching classes) and its
 * lights/vision start, which is too heavy for a unit test.
 *
 * <p>The scan mutates the GLOBAL static registry, so every test here starts
 * from a clean registry ({@link #cleanRegistry()} in {@link Before}) and
 * cleans up after itself ({@link After}) via the finder's
 * {@code beforeUnload}, so no registration leaks into other tests. No other
 * test touches DaemonManager (verified), so the cleanup cannot wipe
 * anyone else's registrations.
 *
 * <p>Load-order convention (same as OpModeStorageSimTest and
 * RobotDashboardSimTest): the sim override lives in this class's static
 * initializer and this class is listed right after RobotDashboardSimTest in
 * UnitTestAll, so the override is set before any test method runs.
 */
public class RobotDaemonsSimTest {

    static {
        // Must run before any test method (see class javadoc).
        SimEnvironment.setOverride(true);
    }

    @AfterClass
    public static void restoreAutoDetect() {
        SimEnvironment.setOverride(null);
    }

    /** The 11 robot daemons, in ROBOT_DAEMON_ORDER (test pin). */
    private static final List<String> EXPECTED_ORDER = Arrays.asList(
            "robot-clock", "robot-battery", "robot-teleop-drive", "robot-vision",
            "robot-hive-state", "robot-aimer", "robot-follower", "robot-smart-control",
            "robot-mechanisms", "robot-lights", "robot-debug");

    @Before
    public void cleanRegistryBefore() {
        cleanRegistry();
    }

    @After
    public void cleanRegistryAfter() {
        cleanRegistry();
    }

    private static void cleanRegistry() {
        DaemonManager.DaemonFinder.INSTANCE.beforeUnload(Robot.class.getClassLoader());
    }

    private static List<String> registeredNames() {
        List<String> names = new ArrayList<>();
        for (DaemonManager.RegisteredDaemon daemon : DaemonManager.unmodifiable()) {
            names.add(daemon.name);
        }
        return names;
    }

    private static long countNamed(String name) {
        return DaemonManager.unmodifiable().stream()
                .filter(d -> d.name.equals(name))
                .count();
    }

    @Test
    public void registryIsEmptyWithoutScanInSim() {
        // The harness JVM never runs the Sloth scan either, so it reproduces
        // the sim's empty registry. Without the explicit scan, the lookup
        // throws exactly the error Shlok saw at START in the sim.
        assertFalse(registeredNames().contains("robot-clock"));
        try {
            Robot.orderedRobotDaemons();
            fail("expected IllegalStateException for the missing robot-clock daemon");
        } catch (IllegalStateException e) {
            assertEquals("Expected one robot-clock daemon; found 0", e.getMessage());
        }
    }

    @Test
    public void ensureRegistersAllElevenDaemonsInSim() {
        Robot.ensureDaemonsRegistered();
        // The ordered lookup is the production behavior: without the scan
        // it throws IllegalStateException here (the negative control).
        List<String> orderedNames = Robot.orderedRobotDaemons().stream()
                .map(d -> d.name)
                .collect(Collectors.toList());
        assertEquals(EXPECTED_ORDER, orderedNames);
        List<String> names = registeredNames();
        for (String expected : EXPECTED_ORDER) {
            assertEquals("expected exactly one " + expected + " daemon",
                    1, countNamed(expected));
        }
        assertEquals(EXPECTED_ORDER.size(), names.size());
    }

    @Test
    public void ensureIsIdempotentInSim() {
        // Repeated starts in one sim JVM (several OpMode runs) must not
        // duplicate registrations in the global registry.
        Robot.ensureDaemonsRegistered();
        Robot.ensureDaemonsRegistered();
        Robot.ensureDaemonsRegistered();
        // Without the scan this throws IllegalStateException (negative control).
        Robot.orderedRobotDaemons();
        assertEquals(1, countNamed("robot-clock"));
        assertEquals(EXPECTED_ORDER.size(), registeredNames().size());
    }

    @Test
    public void ensureIsNoopOffSim() {
        // The robot path is untouched: with the sim override off, no scan
        // runs and the registry stays empty.
        try {
            SimEnvironment.setOverride(false);
            Robot.ensureDaemonsRegistered();
            assertFalse(registeredNames().contains("robot-clock"));
        } finally {
            SimEnvironment.setOverride(true);
        }
    }
}
