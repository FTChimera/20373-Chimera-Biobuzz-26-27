package org.firstinspires.ftc.teamcode.util.game;

import static org.junit.Assert.assertEquals;

import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.teamcode.util.AllianceColor;
import org.firstinspires.ftc.teamcode.util.poses.Pose2d;
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
    public void permitsPathsThatOnlyTouchObstacleBoundary() {
        GeometryFactory geometryFactory = new GeometryFactory();
        Polygon obstacle = geometryFactory.createPolygon(new Coordinate[]{
                new Coordinate(0, 0), new Coordinate(10, 0), new Coordinate(10, 10),
                new Coordinate(0, 10), new Coordinate(0, 0)
        });

        List<Pose2d> path = new DijkstraPathfinder().findPath(
                new Pose2d(-1, 0), new Pose2d(11, 0), obstacle);

        assertEquals(2, path.size());
    }

    @Test
    public void returnsNoPathWhenStartIsInsideForbiddenSpace() {
        GeometryFactory geometryFactory = new GeometryFactory();
        Polygon obstacle = geometryFactory.createPolygon(new Coordinate[]{
                new Coordinate(0, 0), new Coordinate(10, 0), new Coordinate(10, 10),
                new Coordinate(0, 10), new Coordinate(0, 0)
        });

        List<Pose2d> path = new DijkstraPathfinder().findPath(
                new Pose2d(5, 5), new Pose2d(11, 0), obstacle);

        assertEquals(0, path.size());
    }
}
