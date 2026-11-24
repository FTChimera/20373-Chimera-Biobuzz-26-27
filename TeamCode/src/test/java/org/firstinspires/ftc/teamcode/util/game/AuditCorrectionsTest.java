package org.firstinspires.ftc.teamcode.util.game;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import com.rmsftc.utils.AllianceColor;
import com.rmsftc.utils.math.Pose;
import org.junit.Test;
import org.locationtech.jts.geom.Coordinate;
import org.locationtech.jts.geom.GeometryFactory;
import org.locationtech.jts.geom.Polygon;

import java.util.List;

public class AuditCorrectionsTest {
    @Test
    public void detectsYellowFromRedAndGreenEvidence() {
        NormalizedRGBA yellow = new NormalizedRGBA();
        yellow.red = 0.45f;
        yellow.green = 0.45f;
        yellow.blue = 0.10f;

        assertEquals(BallCol.POLLEN_YELLOW, BallCol.detect(yellow, 0.06));
    }

    @Test
    public void doesNotClassifyGreenDominanceAsYellow() {
        NormalizedRGBA green = new NormalizedRGBA();
        green.red = 0.01f;
        green.green = 0.90f;
        green.blue = 0.09f;

        assertEquals(BallCol.UNKNOWN, BallCol.detect(green, 0.06));
    }

    @Test(expected = IllegalArgumentException.class)
    public void rejectsUnknownAprilTagAlliance() {
        new BiobuzzAprilTags(AllianceColor.UNKNOWN);
    }

    @Test
    public void routesAroundObstacleCornerWithoutEnteringIt() {
        // Dijkstra-era version of this test asserted a path that only touched
        // the obstacle boundary (start (-1, 0) to end (11, 0) along y = 0).
        // A grid pathfinder snaps endpoints to cell centers, so the equivalent
        // check is a tight route around the corner that never enters the
        // obstacle: start and goal sit one cell out from the blocked corner.
        GeometryFactory geometryFactory = new GeometryFactory();
        Polygon obstacle = geometryFactory.createPolygon(new Coordinate[]{
                new Coordinate(0, 0), new Coordinate(10, 0), new Coordinate(10, 10),
                new Coordinate(0, 10), new Coordinate(0, 0)
        });

        List<Pose> path = new AStarPathfinder().findPath(
                new Pose(15, 3), new Pose(3, 15), obstacle);

        assertFalse(path.isEmpty());
        assertEquals(15.0, path.get(0).x(), 1e-9);
        assertEquals(3.0, path.get(0).y(), 1e-9);
        assertEquals(3.0, path.get(path.size() - 1).x(), 1e-9);
        assertEquals(15.0, path.get(path.size() - 1).y(), 1e-9);
        for (Pose waypoint : path) {
            assertFalse("Waypoint " + waypoint + " entered the obstacle",
                    obstacle.contains(geometryFactory.createPoint(
                            new Coordinate(waypoint.x(), waypoint.y()))));
        }
        // The route around the corner must stay near the direct line, not
        // wander off across the field.
        double pathLength = 0.0;
        for (int i = 0; i < path.size() - 1; i++) {
            pathLength += path.get(i).distance(path.get(i + 1));
        }
        double straight = Math.hypot(15.0 - 3.0, 3.0 - 15.0);
        assertTrue("Path length " + pathLength + " is wildly longer than straight " + straight,
                pathLength < 1.5 * straight);
    }

    @Test
    public void returnsNoPathWhenStartIsInsideForbiddenSpace() {
        GeometryFactory geometryFactory = new GeometryFactory();
        Polygon obstacle = geometryFactory.createPolygon(new Coordinate[]{
                new Coordinate(0, 0), new Coordinate(10, 0), new Coordinate(10, 10),
                new Coordinate(0, 10), new Coordinate(0, 0)
        });

        List<Pose> path = new AStarPathfinder().findPath(
                new Pose(5, 5), new Pose(11, 0), obstacle);

        assertEquals(0, path.size());
    }
}
