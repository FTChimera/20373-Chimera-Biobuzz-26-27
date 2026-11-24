package org.firstinspires.ftc.teamcode.mechanisms;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertTrue;

import com.rmsftc.utils.AllianceColor;
import com.rmsftc.utils.math.Pose;

import org.firstinspires.ftc.teamcode.fakeHw.FakeHardwareMap;
import org.firstinspires.ftc.teamcode.util.SimEnvironment;
import org.firstinspires.ftc.teamcode.util.game.BallCol;
import org.firstinspires.ftc.teamcode.util.game.Field;
import org.junit.After;
import org.junit.Test;

/** Sim-only fakes: detection, stub Vision (no Limelight), stub Lights (no Prism). */
public class SimFakesTest {

    @After
    public void restoreAutoDetect() {
        SimEnvironment.setOverride(null);
        System.clearProperty("ftcsim.jvm");
    }

    // ---- SimEnvironment ----

    @Test
    public void overrideForcesSimOnAndOff() {
        SimEnvironment.setOverride(true);
        assertTrue(SimEnvironment.isSimulation());
        SimEnvironment.setOverride(false);
        assertFalse(SimEnvironment.isSimulation());
    }

    @Test
    public void systemPropertyDetectsSim() {
        SimEnvironment.setOverride(null);
        System.setProperty("ftcsim.jvm", "true");
        assertTrue(SimEnvironment.isSimulation());
        System.clearProperty("ftcsim.jvm");
        assertFalse(SimEnvironment.isSimulation());
    }

    @Test
    public void autoDetectDefaultsToRealHardware() {
        // No ftcsim.jvm property set and no ftcsim DesktopSdk class on the
        // test classpath, so this must read as "on the robot".
        SimEnvironment.setOverride(null);
        System.clearProperty("ftcsim.jvm");
        assertFalse(SimEnvironment.isSimulation());
    }

    // ---- StubVision ----

    @Test
    public void stubVisionConstructsWithNoHardware() {
        // Null map: the stub ignores the map entirely. Fake map: mirrors a
        // real HardwareMap that has no Limelight entry (get() would throw).
        Vision red = Vision.stub(null, AllianceColor.RED);
        Vision redFakeMap = Vision.stub(new FakeHardwareMap(null, null), AllianceColor.RED);
        Vision blue = Vision.stub(new FakeHardwareMap(null, null), AllianceColor.BLUE);
        assertNull(red.limelight3A);
        assertNull(red.camTransform);
        assertNull(redFakeMap.limelight3A);
        assertNull(blue.limelight3A);
    }

    @Test
    public void stubVisionSeesNoTags() {
        Vision vision = Vision.stub(null, AllianceColor.RED);
        assertFalse(vision.anyFiducial());
        assertFalse(vision.anyAllianceFiducial());
        assertFalse(vision.anyAllianceFiducialAfter(System.nanoTime()));
        assertEquals("none", vision.visibleTagIds());
        assertTrue(vision.fiducials().isEmpty());
        assertFalse(vision.hasReliableLocalization());
        assertEquals(-1.0, vision.getDistance(), 0.0);
        assertTrue(Double.isNaN(vision.getBestTx(Field.HiveState.UNTIPPED)));
        assertFalse(vision.isLineupWithinTolerance(Field.HiveState.UNTIPPED, 5.0));
        assertEquals(0.0, vision.getAveragedTx(Field.HiveState.UNTIPPED), 0.0);
    }

    @Test
    public void stubVisionLifecycleAndDebugAreSafe() {
        Vision vision = Vision.stub(null, AllianceColor.RED);
        vision.start();
        vision.update(Math.PI / 4);
        vision.update();
        Pose fieldPose = vision.getFieldPoseEstimate();
        assertEquals(0.0, fieldPose.x(), 0.0);
        assertEquals(0.0, fieldPose.y(), 0.0);
        assertEquals(0.0, fieldPose.heading(), 0.0);
        assertEquals(Vision.RED_PIPELINE_INDEX, vision.getDesiredPipelineIndex());
        assertEquals(-1, vision.getActivePipelineIndex());
        assertNull(vision.getLatestResult());
        assertNull(vision.getBotposeRaw());
        assertNull(vision.getInferredHiveState());
        assertTrue(vision.getDebugString().contains("SIM stub vision"));
        assertTrue(vision.getDebugString().contains("Alliance=RED"));
        vision.stop();
    }

    @Test
    public void stubVisionBlueReportsBluePipeline() {
        Vision vision = Vision.stub(null, AllianceColor.BLUE);
        assertEquals(Vision.BLUE_PIPELINE_INDEX, vision.getDesiredPipelineIndex());
        assertTrue(vision.getDebugString().contains("Alliance=BLUE"));
    }

    @Test
    public void stubVisionApplyHiveStateIsNoop() {
        Vision vision = Vision.stub(null, AllianceColor.RED);
        Field field = new Field(AllianceColor.RED, 17, 17);
        vision.applyInferredHiveState(field);
        assertNull(vision.getInferredHiveState());
    }

    // ---- StubLights ----

    @Test
    public void stubLightsConstructsWithNoHardware() {
        Lights lights = Lights.stub(null, AllianceColor.RED);
        Lights lightsFakeMap = Lights.stub(new FakeHardwareMap(null, null), AllianceColor.BLUE);
        assertNull(lights.prism);
        assertNull(lightsFakeMap.prism);
        assertEquals("SIM stub lights", lights.getDebugString());
    }

    @Test
    public void stubLightsEveryPublicMethodIsSafe() {
        Lights lights = Lights.stub(null, AllianceColor.RED);
        lights.start();
        lights.update();
        lights.displayTeamColor();
        lights.rainbowParty();
        lights.displayGreen();
        lights.flashRed();
        lights.oneMinThing();
        lights.off();
        lights.clearAll();
        lights.stopEventAnimation();
        lights.onReZero();
        lights.onIntakeOn();
        lights.onIntakeOff();
        lights.onDeployerToggle(true);
        lights.onDeployerToggle(false);
        lights.onEjectStart();
        lights.onShootStart();
        lights.onReverseOuttakeStart();
        lights.onPanicStart();
        lights.onParkStart();
        lights.displayTransferColor(BallCol.POLLEN_YELLOW, true);
        lights.displayTransferColor(BallCol.NECTAR_RED, false);
        lights.displayTransferColor(BallCol.UNKNOWN, false);
        lights.displayAllianceColor();
        lights.stop();
        assertEquals("SIM stub lights", lights.getDebugString());
    }
}
