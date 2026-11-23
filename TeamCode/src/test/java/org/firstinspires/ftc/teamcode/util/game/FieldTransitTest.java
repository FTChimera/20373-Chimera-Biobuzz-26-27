package org.firstinspires.ftc.teamcode.util.game;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;

import com.pedropathing.paths.Path;
import com.rmsftc.utils.AllianceColor;

import org.firstinspires.ftc.teamcode.planners.FieldCoordinates;
import org.firstinspires.ftc.teamcode.planners.ParkAutoStartPoses;
import org.firstinspires.ftc.teamcode.util.poses.Pose2d;
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
            Pose2d garden = color == AllianceColor.RED
                    ? new Pose2d(40, 10, Math.toRadians(65))
                    : new Pose2d(104, 134, Math.toRadians(245));
            Pose2d loading = color == AllianceColor.RED
                    ? new Pose2d(60, 120, -Math.PI / 2)
                    : new Pose2d(84, 24, Math.PI / 2);
            assertClear(field.generateTransitPath(garden, loading));
            assertClear(field.generateTransitPath(loading, garden));
        }
    }

    @Test
    public void loadingStartParksOnItsOwnAllianceSide() {
        Field red = new Field(AllianceColor.RED, 17, 17);
        Path redPath = red.generatePathToLoadingZoneShortest(new Pose2d(64, 128, -Math.PI / 2), 3);
        assertEquals(true, red.isRobotOverlappingLoadingZone(new Pose2d(redPath.endPose())));
        assertParkClear(redPath);
        Field blue = new Field(AllianceColor.BLUE, 17, 17);
        Path bluePath = blue.generatePathToLoadingZoneShortest(new Pose2d(80, 16, Math.PI / 2), 3);
        assertEquals(true, blue.isRobotOverlappingLoadingZone(new Pose2d(bluePath.endPose())));
        assertParkClear(bluePath);
    }

    @Test(expected = Field.NoRouteException.class)
    public void rejectsTransitStartingInsideAHive() {
        new Field(AllianceColor.RED, 17, 17).generateTransitPath(
                new Pose2d(48, 72, 0), new Pose2d(18, 108, 0));
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
        }
    }
}
