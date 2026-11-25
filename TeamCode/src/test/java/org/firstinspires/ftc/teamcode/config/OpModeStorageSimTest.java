package org.firstinspires.ftc.teamcode.config;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import com.rmsftc.utils.AllianceColor;
import com.rmsftc.utils.MatchMode;
import com.rmsftc.utils.math.Pose;

import org.firstinspires.ftc.teamcode.planners.Constants;
import org.firstinspires.ftc.teamcode.testing.FtcSimulationRule;
import org.junit.Rule;
import org.junit.Test;

/**
 * Regression test for the ftcsim OpModeStorage crash: on a desktop JVM the
 * class's static initializer reads the stored alliance, which used to build
 * FTCUtils Storage (SharedPreferences via AppUtil) and throw, poisoning the
 * class for the whole JVM — every later use then fails with
 * NoClassDefFoundError: Could not initialize class OpModeStorage.
 *
 * <p>The scoped simulation rule enables desktop storage before its first
 * reference and restores values after every test, independently of suite order.
 */
public class OpModeStorageSimTest {

    private static final double EPSILON = 1e-9;

    @Rule public final FtcSimulationRule simulation = new FtcSimulationRule();

    @Test
    public void classLoadsAndDefaultsAreInMemoryValuesInSim() {
        // Merely referencing the class runs its static initializer; without
        // the sim fallback that throw would error this test at load time.
        assertEquals(AllianceColor.BLUE, OpModeStorage.allianceColor.get());
        assertEquals(MatchMode.QUALIFICATIONS, OpModeStorage.matchMode.get());
        assertFalse(OpModeStorage.initialized.get());
        assertPoseEquals(Constants.StartPoses.gardenArea, OpModeStorage.pose.get());
    }

    @Test
    public void storedValuesRoundTripInMemory() {
        OpModeStorage.allianceColor.set(AllianceColor.RED);
        OpModeStorage.matchMode.set(MatchMode.PLAYOFFS);
        OpModeStorage.pose.set(Constants.StartPoses.loadingArea);

        assertEquals(AllianceColor.RED, OpModeStorage.allianceColor.get());
        assertEquals(MatchMode.PLAYOFFS, OpModeStorage.matchMode.get());
        assertPoseEquals(Constants.StartPoses.loadingArea, OpModeStorage.pose.get());

        OpModeStorage.allianceColor.set(AllianceColor.BLUE);
        OpModeStorage.matchMode.set(MatchMode.QUALIFICATIONS);
        OpModeStorage.pose.set(Constants.StartPoses.gardenArea);
    }

    @Test
    public void setEndValsAndEndMatchWorkInSim() {
        OpModeStorage.setEndVals(Constants.StartPoses.loadingArea);
        assertPoseEquals(Constants.StartPoses.loadingArea, OpModeStorage.pose.get());
        assertTrue(OpModeStorage.initialized.get());

        // endMatch clears only the in-memory flag; the stored pose survives.
        OpModeStorage.endMatch();
        assertFalse(OpModeStorage.initialized.get());
        assertPoseEquals(Constants.StartPoses.loadingArea, OpModeStorage.pose.get());

        OpModeStorage.pose.set(Constants.StartPoses.gardenArea);
    }

    private static void assertPoseEquals(Pose expected, Pose actual) {
        assertEquals(expected.x(), actual.x(), EPSILON);
        assertEquals(expected.y(), actual.y(), EPSILON);
        assertEquals(expected.heading(), actual.heading(), EPSILON);
    }
}
