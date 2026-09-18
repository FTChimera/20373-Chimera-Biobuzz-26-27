package org.firstinspires.ftc.teamcode.library;

import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;
import org.junit.Test;

import java.util.Arrays;
import java.util.Collections;
import java.util.List;

import static org.firstinspires.ftc.teamcode.library.IntakeBallPath.AREA_DISTANCE_SCALE;
import static org.firstinspires.ftc.teamcode.library.math.MathUtils.EPSILON;
import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertTrue;

public class IntakeBallPathTest {
    @Test
    public void publicGeneratorRejectsEmptyInputsBeforeUsingHardware() {
        assertNull(IntakeBallPath.generateIntakePathFromCam(null, new Pose(), null, 0, Collections.emptyList()));
        assertNull(IntakeBallPath.generateIntakePathFromCam(null, new Pose(), null, 2, null));
        assertNull(IntakeBallPath.generateIntakePathFromCam(null, new Pose(), null, 2, Collections.emptyList()));
    }

    @Test
    public void estimateForwardDistanceUsesTyFirstAndFallsBackToArea() {
        assertEquals(1.4, IntakeBallPath.estimateForwardDistanceInches(10.0, 0.36, 0.0, 35.0), 1e-6);
        assertEquals(AREA_DISTANCE_SCALE / Math.sqrt(0.25),
                IntakeBallPath.estimateForwardDistanceInches(0.0, 0.25, 0.0, 0.0),
                EPSILON);
    }

    @Test
    public void buildCandidatesFiltersDuplicatesAndSortsByForwardDistance() {
        List<IntakeBallPath.DetectionReading> detections = Arrays.asList(
                detection(90.0, 36.0, 0.0, 0.0),
                detection(99.0, 40.0, 0.0, 0.0),
                detection(80.0, 25.0, 7.0, 0.0),
                detection(20.0, 50.0, 0.0, 0.0),
                detection(90.0, 25.0, 30.0, 0.0)
        );

        List<IntakeBallPath.DetectionCandidate> candidates = IntakeBallPath.buildCandidates(new Pose(), detections, 0.0, 0.0);

        assertEquals(2, candidates.size());
        assertEquals(18.0 / Math.sqrt(0.40), candidates.get(0).forwardDistance, 1e-6);
        assertEquals(36.0, candidates.get(1).forwardDistance, 1e-6);
        assertEquals(36.0 * Math.tan(Math.toRadians(7.0)), candidates.get(1).lateralOffset, 1e-6);
        assertTrue(candidates.get(0).qualityScore > candidates.get(1).qualityScore);
    }

    @Test
    public void buildCandidatesEnforcesDetectionAndDistanceBounds() {
        List<IntakeBallPath.DetectionCandidate> candidates = IntakeBallPath.buildCandidates(
                new Pose(),
                Arrays.asList(
                        detection(35.0, 15.0, 0.0, 0.0),
                        detection(34.99, 40.0, 0.0, 0.0),
                        detection(90.0, 14.99, 0.0, 0.0),
                        detection(90.0, 40.0, 0.0, 80.0),
                        detection(90.0, 40.0, 0.0, 1.0),
                        detection(90.0, 40.0, 90.0, 0.0)
                ),
                0.0,
                0.0
        );

        assertEquals(1, candidates.size());
        assertEquals(AREA_DISTANCE_SCALE / Math.sqrt(0.15), candidates.get(0).forwardDistance, EPSILON);
        assertTrue(Double.isFinite(candidates.get(0).fieldPose.getX()));
        assertTrue(Double.isFinite(candidates.get(0).fieldPose.getY()));
    }

    @Test
    public void sequenceSelectionBalancesQualityWithTurnAndGapPenalties() {
        IntakeBallPath.DetectionCandidate candidateA =
                new IntakeBallPath.DetectionCandidate(new Pose(12.0, 0.0), 12.0, 0.0, 0.60);
        IntakeBallPath.DetectionCandidate candidateB =
                new IntakeBallPath.DetectionCandidate(new Pose(24.0, 1.0), 24.0, 1.0, 0.55);
        IntakeBallPath.DetectionCandidate candidateC =
                new IntakeBallPath.DetectionCandidate(new Pose(25.0, 8.0), 25.0, 8.0, 0.95);

        List<IntakeBallPath.DetectionCandidate> chosen = IntakeBallPath.selectBestSequence(
                Arrays.asList(candidateA, candidateB, candidateC),
                2
        );
        assertEquals(2, chosen.size());
        assertEquals(12.0, chosen.get(0).forwardDistance, EPSILON);
        assertEquals(24.0, chosen.get(1).forwardDistance, EPSILON);

        List<IntakeBallPath.DetectionCandidate> fallback = IntakeBallPath.selectBestSequence(
                Arrays.asList(candidateA, candidateB),
                3
        );
        assertEquals(1, fallback.size());
        assertEquals(12.0, fallback.get(0).forwardDistance, EPSILON);

        assertTrue(IntakeBallPath.selectBestSequence(Collections.emptyList(), 1).isEmpty());
        assertTrue(IntakeBallPath.selectBestSequence(Arrays.asList(candidateA, candidateB), 0).isEmpty());
    }

    @Test
    public void pathStrategyChecksAndBuildersProduceExpectedChains() {
        List<Pose> gentleWaypoints = Arrays.asList(
                new Pose(0.0, 0.0),
                new Pose(10.0, 0.0),
                new Pose(20.0, 2.0)
        );
        List<Pose> sharpWaypoints = Arrays.asList(
                new Pose(0.0, 0.0),
                new Pose(10.0, 0.0),
                new Pose(10.0, 10.0)
        );
        List<Pose> unevenWaypoints = Arrays.asList(
                new Pose(0.0, 0.0),
                new Pose(2.0, 0.0),
                new Pose(20.0, 0.0)
        );

        assertTrue(IntakeBallPath.shouldUsePedroCurveThrough(gentleWaypoints));
        assertFalse(IntakeBallPath.shouldUsePedroCurveThrough(sharpWaypoints));
        assertFalse(IntakeBallPath.shouldUsePedroCurveThrough(unevenWaypoints));

        PathChain adaptive = IntakeBallPath.buildAdaptiveHandlePathChain(Arrays.asList(
                new Pose(0.0, 0.0),
                new Pose(10.0, 0.0)
        ));
        assertNotNull(adaptive);
        assertEquals(1, adaptive.size());
        assertEquals(PathChain.DecelerationType.LAST_PATH, adaptive.getDecelerationType());
        assertEquals(0.0, adaptive.getPath(0).getFirstControlPoint().getX(), EPSILON);
        assertEquals(10.0, adaptive.getPath(0).getLastControlPoint().getX(), EPSILON);

        assertNull(IntakeBallPath.buildAdaptiveHandlePathChain(Arrays.asList(
                new Pose(0.0, 0.0),
                new Pose(0.0, 0.0)
        )));

        PathChain pedro = IntakeBallPath.buildPedroCurveThroughPathChain(new PathBuilder(null), gentleWaypoints);
        assertNotNull(pedro);
        assertEquals(2, pedro.size());
        assertEquals(PathChain.DecelerationType.LAST_PATH, pedro.getDecelerationType());
        assertEquals(20.0, pedro.endPoint().getX(), 1e-6);
        assertEquals(2.0, pedro.endPoint().getY(), 1e-6);
    }

    private IntakeBallPath.DetectionReading detection(double confidence, double targetArea, double targetXDegrees, double targetYDegrees) {
        return new IntakeBallPath.DetectionReading(confidence, targetArea, targetXDegrees, targetYDegrees);
    }
}
