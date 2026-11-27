package org.firstinspires.ftc.teamcode.testing;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

import com.rmsftc.utils.AllianceColor;

import org.firstinspires.ftc.teamcode.config.OpModeStorage;
import org.firstinspires.ftc.teamcode.mechanisms.Shooter;
import org.firstinspires.ftc.teamcode.util.SimEnvironment;
import org.junit.Rule;
import org.junit.Test;
import org.junit.runner.Description;
import org.junit.runners.model.Statement;

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

}
