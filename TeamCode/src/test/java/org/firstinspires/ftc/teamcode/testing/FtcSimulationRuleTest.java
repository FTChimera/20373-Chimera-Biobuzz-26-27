package org.firstinspires.ftc.teamcode.testing;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

import com.rmsftc.utils.AllianceColor;
import com.rmsftc.utils.ftc.DaemonManager;
import com.rmsftc.utils.ftc.daemons.FTCDaemon;

import org.firstinspires.ftc.teamcode.config.OpModeStorage;
import org.firstinspires.ftc.teamcode.mechanisms.Shooter;
import org.firstinspires.ftc.teamcode.util.SimEnvironment;
import org.junit.Rule;
import org.junit.Test;
import org.junit.runner.Description;
import org.junit.runners.model.Statement;

import java.util.ArrayList;
import java.util.List;

public class FtcSimulationRuleTest {
    @Rule public final FtcSimulationRule simulation = new FtcSimulationRule();

    @Test
    public void restoresSharedStateAndEnvironmentAfterFailure() throws Throwable {
        AllianceColor alliance = OpModeStorage.allianceColor.get();
        boolean sotm = Shooter.useSotm.get();
        Boolean override = SimEnvironment.override;
        String property = System.getProperty("ftcsim.jvm");
        AssertionError expected = new AssertionError("intentional test failure");
        Statement failing = new Statement() {
            @Override public void evaluate() {
                OpModeStorage.allianceColor.set(AllianceColor.RED);
                Shooter.useSotm.set(!sotm);
                SimEnvironment.setOverride(false);
                System.clearProperty("ftcsim.jvm");
                throw expected;
            }
        };
        try {
            new FtcSimulationRule().apply(failing, Description.EMPTY).evaluate();
            fail("The original test failure must propagate");
        } catch (AssertionError actual) {
            assertSame(expected, actual);
        }
        assertEquals(alliance, OpModeStorage.allianceColor.get());
        assertEquals(sotm, Shooter.useSotm.get());
        assertEquals(override, SimEnvironment.override);
        assertEquals(property, System.getProperty("ftcsim.jvm"));
        assertTrue(SimEnvironment.isSimulation());
    }

    @Test
    public void restoresDaemonRegistrationAfterFailure() throws Throwable {
        List<DaemonManager.RegisteredDaemon> previous = new ArrayList<>(DaemonManager.unmodifiable());
        AssertionError expected = new AssertionError("intentional daemon test failure");
        Statement failing = new Statement() {
            @Override public void evaluate() {
                DaemonManager.DaemonFinder.INSTANCE.scan(FtcSimulationRuleTest.class.getClassLoader(),
                        FtcSimulationRuleTest.class);
                assertTrue(DaemonManager.unmodifiable().stream()
                        .anyMatch(daemon -> daemon.name.equals("test-scoped-daemon")));
                throw expected;
            }
        };
        try {
            new FtcSimulationRule().apply(failing, Description.EMPTY).evaluate();
            fail("The original test failure must propagate");
        } catch (AssertionError actual) {
            assertSame(expected, actual);
        }
        assertEquals(previous, new ArrayList<>(DaemonManager.unmodifiable()));
    }

    @FTCDaemon(name = "test-scoped-daemon")
    public static void scopedDaemon() { }
}
