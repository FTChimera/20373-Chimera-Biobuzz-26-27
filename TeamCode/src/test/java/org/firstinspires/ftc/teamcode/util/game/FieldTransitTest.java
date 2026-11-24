package org.firstinspires.ftc.teamcode.util.game;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;

import com.pedropathing.paths.Path;
import com.rmsftc.utils.AllianceColor;

import org.firstinspires.ftc.teamcode.planners.FieldCoordinates;
import org.firstinspires.ftc.teamcode.planners.ParkAutoStartPoses;
import com.rmsftc.utils.math.Pose;
import org.junit.Test;
import org.locationtech.jts.geom.Coordinate;
import org.locationtech.jts.geom.Geometry;

public class FieldTransitTest {
    @Test
    public void allParkStartsClearHiveBasesWithTheWholeBody() {
        assertParkClear(new Field(AllianceColor.RED, 17, 17).generateTransitPath(
                ParkAutoStartPoses.redUntippedStart(), ParkAutoStartPoses.destination(AllianceColor.RED)));
        assertParkClear(new Field(AllianceColor.RED, 17, 17).generateTransitPath(
                ParkAutoStartPoses.redTippedStart(), ParkAutoStartPoses.destination(AllianceColor.RED)));
        assertParkClear(new Field(AllianceColor.BLUE, 17, 17).generateTransitPath(
                ParkAutoStartPoses.blueUntippedStart(), ParkAutoStartPoses.destination(AllianceColor.BLUE)));
        assertParkClear(new Field(AllianceColor.BLUE, 17, 17).generateTransitPath(
                ParkAutoStartPoses.blueTippedStart(), ParkAutoStartPoses.destination(AllianceColor.BLUE)));
    }

    @Test
    public void shootingPoseTransfersClearHiveBasesInBothDirectionsAndAlliances() {
        for (AllianceColor color : new AllianceColor[]{AllianceColor.RED, AllianceColor.BLUE}) {
            Field field = new Field(color, 17, 17);
            Pose garden = color == AllianceColor.RED
                    ? new Pose(40, 10, Math.toRadians(65))
                    : new Pose(104, 134, Math.toRadians(245));
            Pose loading = color == AllianceColor.RED
                    ? new Pose(60, 120, -Math.PI / 2)
                    : new Pose(84, 24, Math.PI / 2);
            assertClear(field.generateTransitPath(garden, loading));
            assertClear(field.generateTransitPath(loading, garden));
        }
    }

    @Test
    public void loadingStartParksOnItsOwnAllianceSide() {
        Field red = new Field(AllianceColor.RED, 17, 17);
        Path redPath = red.generatePathToLoadingZoneShortest(new Pose(64, 128, -Math.PI / 2), 3);
        assertEquals(true, red.isRobotOverlappingLoadingZone(Pose.fromPedro(redPath.endPose())));
        assertParkClear(redPath);
        Field blue = new Field(AllianceColor.BLUE, 17, 17);
        Path bluePath = blue.generatePathToLoadingZoneShortest(new Pose(80, 16, Math.PI / 2), 3);
        assertEquals(true, blue.isRobotOverlappingLoadingZone(Pose.fromPedro(bluePath.endPose())));
        assertParkClear(bluePath);
    }

    @Test(expected = Field.NoRouteException.class)
    public void rejectsTransitStartingInsideAHive() {
        new Field(AllianceColor.RED, 17, 17).generateTransitPath(
                new Pose(48, 72, 0), new Pose(18, 108, 0));
    }

    @Test
    public void passOneReprosRouteWithRealFieldGeometry() {
        // Pass-1 fuzz failures: the exact poses are legal (outside the
        // inflated hive footprint) but their cell centers are blocked, so the
        // old code returned empty. The nearest-free-cell fallback must route
        // them now.
        Field red = new Field(AllianceColor.RED, 17, 17);
        assertClear(red.generateTransitPath(new Pose(31.5, 72, 0), new Pose(120, 72, 0)));
        assertClear(red.generateTransitPath(new Pose(31, 41, 0), new Pose(120, 72, 0)));
    }

    @Test(expected = Field.NoRouteException.class)
    public void goalInsideInflatedFlowerFootprintHasNoRoute() {
        // Pass-1 repro goal (100, 10) does NOT route, correctly: it sits
        // 5.2 in from the raw south-flower footprint, inside the 13.02 in
        // inflation (half-diagonal 12.02 + 1.0 clearance), so the exact goal
        // pose itself is illegal and no fallback can save it.
        new Field(AllianceColor.RED, 17, 17).generateTransitPath(
                new Pose(60, 120, 0), new Pose(100, 10, 0));
    }

    @Test
    public void nearbyLegalGoalRoutes() {
        // Same area as the (100, 10) repro, but 10 in north of it: outside
        // the inflated south-flower footprint, so a route must exist.
        Field red = new Field(AllianceColor.RED, 17, 17);
        assertClear(red.generateTransitPath(new Pose(60, 120, 0), new Pose(100, 20, 0)));
    }

    private static void assertClear(Path path) {
        assertClear(path, false);
    }

    private static void assertParkClear(Path path) {
        assertClear(path, true);
    }

    private static void assertClear(Path path, boolean checkWalls) {
        Geometry bases = Field.geometryFactory.createPolygon(FieldCoordinates.westHiveBase())
                .union(Field.geometryFactory.createPolygon(FieldCoordinates.eastHiveBase()));
        Geometry flowers = Field.geometryFactory.createPolygon(FieldCoordinates.westFlower())
                .union(Field.geometryFactory.createPolygon(FieldCoordinates.northFlower()))
                .union(Field.geometryFactory.createPolygon(FieldCoordinates.eastFlower()))
                .union(Field.geometryFactory.createPolygon(FieldCoordinates.southFlower()));
        for (int i = 0; i <= 1000; i++) {
            com.pedropathing.math.Pose pose = path.get(i / 1000.0);
            Coordinate[] corners = new Coordinate[5];
            double cos = Math.cos(pose.heading());
            double sin = Math.sin(pose.heading());
            double[][] offsets = {{-8.5, -8.5}, {8.5, -8.5}, {8.5, 8.5}, {-8.5, 8.5}};
            for (int j = 0; j < offsets.length; j++) {
                double x = offsets[j][0];
                double y = offsets[j][1];
                corners[j] = new Coordinate(pose.x() + x * cos - y * sin,
                        pose.y() + x * sin + y * cos);
                if (checkWalls) {
                    org.junit.Assert.assertTrue("Body crosses a field wall at " + i / 1000.0,
                            corners[j].x >= -1e-6 && corners[j].x <= 144 + 1e-6
                                    && corners[j].y >= -1e-6 && corners[j].y <= 144 + 1e-6);
                }
            }
            corners[4] = new Coordinate(corners[0]);
            assertFalse("Body intersects a hive at path parameter " + i / 1000.0,
                    bases.intersects(Field.geometryFactory.createPolygon(corners)));
            assertFalse("Body intersects a flower at path parameter " + i / 1000.0,
                    flowers.intersects(Field.geometryFactory.createPolygon(corners)));
        }
    }
}
