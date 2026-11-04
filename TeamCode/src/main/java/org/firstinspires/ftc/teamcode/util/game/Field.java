package org.firstinspires.ftc.teamcode.util.game;

import androidx.annotation.NonNull;

import com.pedropathing.api.Paths;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Vector2D;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.interpolator.Interpolator;

import org.firstinspires.ftc.teamcode.util.poses.PoseFactory;
import org.firstinspires.ftc.teamcode.planners.CommonKt;
import org.firstinspires.ftc.teamcode.util.AllianceColor;
import org.firstinspires.ftc.teamcode.util.poses.Pose2d;
import org.locationtech.jts.geom.Coordinate;
import org.locationtech.jts.geom.Geometry;
import org.locationtech.jts.geom.GeometryFactory;
import org.locationtech.jts.geom.Point;
import org.locationtech.jts.geom.Polygon;
import org.locationtech.jts.operation.buffer.BufferOp;
import org.locationtech.jts.operation.distance.DistanceOp;
import org.locationtech.jts.operation.buffer.BufferParameters;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;

public class Field {
    public static class NoRouteException extends IllegalStateException {
        public NoRouteException(String message) {
            super(message);
        }
    }

    public enum HiveState {
        TIPPED,
        UNTIPPED;

        public Pose goalPose(@NonNull AllianceColor allianceColor) {
            // Representative coordinate positions for the Red and Blue alliance target zones
            if (allianceColor.isRed()) {
                if (this == TIPPED) {
                    return new Pose(60, 84);
                } else {
                    return new Pose(60, 60);
                }
            }
            if (allianceColor.isBlue()) {
                if (this == TIPPED) {
                    return new Pose(84, 60);
                } else {
                    return new Pose(84, 84);
                }
            }
            return Pose.zero();
        }
    }

    private HiveState hiveState;
    public static final GeometryFactory geometryFactory = new GeometryFactory();
    public static final DijkstraPathfinder pathfinder = new DijkstraPathfinder();
    public Polygon loadingZone;
    private Geometry obstacles;
    private Geometry flowerObstacles;

    // Manual safety buffer (inches) applied to the hive bases only when
    // building the forbidden space. Replaces the half-diagonal formula
    // (hypot(botLength, botWidth) / 2 + 1.5 = 12.46 in), which left only a
    // 17.1-in corridor between the hives and made the whole central band
    // unparkable-from. Flowers join the forbidden space raw, unbuffered.
    private static final double SAFETY_RADIUS_INCHES = 5.0;

    // Defined loading zones using 24x24 inch floor tiles corners on standard 144x144 field layout
    public static final Coordinate[] RED_LOADING_ZONE = new Coordinate[]{
            new Coordinate(0, 120),
            new Coordinate(12, 120),
            new Coordinate(12, 96),
            new Coordinate(0, 96),
            new Coordinate(0, 120)
    };
    public static final Coordinate[] BLUE_LOADING_ZONE = new Coordinate[]{
            new Coordinate(144, 24),
            new Coordinate(120, 24),
            new Coordinate(120, 48),
            new Coordinate(144, 48),
            new Coordinate(144, 24)
    };

    public double botWidth, botLength;

    public Field(AllianceColor color, double botWidth, double botLength) {
        hiveState = HiveState.UNTIPPED;
        this.botLength = botLength;
        this.botWidth = botWidth;

        loadingZone = color.isRed() ?
                geometryFactory.createPolygon(RED_LOADING_ZONE) :
                geometryFactory.createPolygon(BLUE_LOADING_ZONE);

        // Model static obstacles on the field (e.g., Hive structure bases modeled as rectangles)
        Polygon hiveBase1 = geometryFactory.createPolygon(new Coordinate[]{
                new Coordinate(45, 50), new Coordinate(51, 50),
                new Coordinate(51, 94), new Coordinate(45, 94), new Coordinate(45, 50)
        });
        Polygon hiveBase2 = geometryFactory.createPolygon(new Coordinate[]{
                new Coordinate(99, 94), new Coordinate(93, 94),
                new Coordinate(93, 50), new Coordinate(99, 50), new Coordinate(99, 94)
        });
        obstacles = hiveBase1.union(hiveBase2);
        // FLAGGED CHANGE: FLOWER footprints, flush against the perimeter walls.
        // Kept separate from `obstacles`: the safety-radius buffer applies to the
        // hive only, so flowers join the forbidden space RAW (see the forbiddenSpace
        // line in generatePathToLoadingZoneShortest).
        Polygon flowerWest = geometryFactory.createPolygon(new Coordinate[]{
                new Coordinate(0, 45), new Coordinate(4.9, 45),
                new Coordinate(4.9, 51), new Coordinate(0, 51), new Coordinate(0, 45)
        });
        Polygon flowerNorth = geometryFactory.createPolygon(new Coordinate[]{
                new Coordinate(45, 139.1), new Coordinate(51, 139.1),
                new Coordinate(51, 144), new Coordinate(45, 144), new Coordinate(45, 139.1)
        });
        Polygon flowerEast = geometryFactory.createPolygon(new Coordinate[]{
                new Coordinate(139.1, 93), new Coordinate(144, 93),
                new Coordinate(144, 99), new Coordinate(139.1, 99), new Coordinate(139.1, 93)
        });
        Polygon flowerSouth = geometryFactory.createPolygon(new Coordinate[]{
                new Coordinate(93, 0), new Coordinate(99, 0),
                new Coordinate(99, 4.9), new Coordinate(93, 4.9), new Coordinate(93, 0)
        });
        flowerObstacles = flowerWest.union(flowerNorth).union(flowerEast).union(flowerSouth);
    }

    public void setHiveState(HiveState set) {
        this.hiveState = set;
    }

    public HiveState getHiveState() {
        return hiveState;
    }

    public double getClosestLoadingZoneDistanceToRobot(Pose2d robotPose) {
        return loadingZone.distance(geometryFactory.createPoint(robotPose.toCoordinate()));
    }

    public boolean isRobotOverlappingLoadingZone(Pose2d robotPose) {
        return loadingZone.intersects(createRobotGeom(robotPose));
    }

    public Geometry createRobotGeom(Pose2d robotPose) {
        double halfLength = botLength / 2;
        double halfWidth = botWidth / 2;
        return geometryFactory.createPolygon(new Coordinate[]{
                new Coordinate(robotPose.x - halfLength, robotPose.y - halfWidth),
                new Coordinate(robotPose.x + halfLength, robotPose.y - halfWidth),
                new Coordinate(robotPose.x + halfLength, robotPose.y + halfWidth),
                new Coordinate(robotPose.x - halfLength, robotPose.y + halfWidth),
                new Coordinate(robotPose.x - halfLength, robotPose.y - halfWidth)
        });
    }

    public Path generatePathToLoadingZoneShortest(Pose2d robotPose, double innerVal) {
        if (isRobotOverlappingLoadingZone(robotPose)) {
            throw new NoRouteException("Robot already overlaps the loading zone; no park path is required");
        }
        // FLAGGED CHANGE: use the closest point PAIR: [0] is on the loading zone,
        // [1] is on the robot rectangle (JTS DistanceOp.nearestPoints contract).
        // The robot only needs its closest point to reach innerVal past the zone
        // boundary; its center no longer has to drive all the way inside.
        Coordinate[] nearest = DistanceOp.nearestPoints(loadingZone, createRobotGeom(robotPose));
        Coordinate closestPtOnLoadingZone = nearest[0];
        Coordinate closestPtOnRobot = nearest[1];
        double gap = closestPtOnLoadingZone.distance(closestPtOnRobot);
        double theta;
        if (gap < 1e-9) {
            // Touching (zero gap): fall back to the center-based direction.
            theta = Vector2D.cartesian(
                    closestPtOnLoadingZone.x - robotPose.x,
                    closestPtOnLoadingZone.y - robotPose.y).theta();
        } else {
            theta = Vector2D.cartesian(
                    closestPtOnLoadingZone.x - closestPtOnRobot.x,
                    closestPtOnLoadingZone.y - closestPtOnRobot.y).theta();
        }
        Vector2D travel = Vector2D.polar(gap + innerVal, theta);
        Pose2d aLittleInwardPose = PoseFactory.of(robotPose.x, robotPose.y, 0).plus(travel.toPose());

        // Convert Pedro Pathing Poses
        Pose2d startPose = PoseFactory.of(robotPose.x, robotPose.y, robotPose.heading);
        Pose2d endPose = PoseFactory.of(aLittleInwardPose.x, aLittleInwardPose.y, aLittleInwardPose.heading);

        // Inflate the hive bases by the manual safety buffer (flowers join raw, unbuffered)
        double safetyRadius = SAFETY_RADIUS_INCHES;
        BufferParameters bufParams = new BufferParameters();
        bufParams.setJoinStyle(BufferParameters.JOIN_MITRE);
        Geometry forbiddenSpace = BufferOp.bufferOp(obstacles, safetyRadius, bufParams)
                .union(flowerObstacles);
        List<Pose2d> poses = pathfinder.findPath(startPose, endPose, forbiddenSpace);

        if (poses.isEmpty()) {
            throw new NoRouteException("No collision-free path to the loading zone");
        }

        // FLAGGED CHANGE: end heading = final approach direction, rounded to the
        // nearest cardinal (0 / 90 / 180 / 270). findPath already preserves the
        // end pose's position; only its heading is updated here, before the
        // heading interpolator is built. This is the forward-case end heading
        // only: when the path arrives reversed, reversingTangent below ignores
        // it and stays reversed to the end.
        if (poses.size() >= 2) {
            Pose2d prev = poses.get(poses.size() - 2);
            Pose2d end = poses.get(poses.size() - 1);
            double approach = Math.atan2(end.y - prev.y, end.x - prev.x);
            end.heading = roundToCardinal(approach);
        }

        // FLAGGED CHANGE: heading interpolator that compares the curve's actual
        // initial tangent to the robot heading and reverses if necessary
        // (CommonKt.reversingTangent), instead of the chord-based
        // fastInterpolation. From Java the t-values must be passed explicitly
        // since Kotlin default arguments do not apply across the boundary.
        Interpolator closestTangent = CommonKt.reversingTangent(
                robotPose.heading, poses.get(poses.size() - 1).heading, 0.7, 0.9);
        return Paths.through(poses.toArray(new Pose2d[0])).heading(closestTangent);
    }

    /** Rounds a heading in radians to the nearest cardinal, returned in [0, 2*PI). */
    private static double roundToCardinal(double headingRad) {
        double deg = Math.toDegrees(headingRad) % 360.0;
        if (deg < 0) deg += 360.0;
        double rounded = Math.round(deg / 90.0) * 90.0 % 360.0;
        return Math.toRadians(rounded);
    }

    /*
    public Path generatePathToLoadingZoneShortest(Pose2d robotPose, double innerVal) {
        Coordinate closestPtOnLoadingZone = DistanceOp.nearestPoints(loadingZone, createRobotGeom(robotPose))[0];
        double theta = Vector2D.cartesian(closestPtOnLoadingZone.x - robotPose.x, closestPtOnLoadingZone.y - robotPose.y).theta();
        Vector2D inwards = Vector2D.polar(innerVal, theta);
        Pose2d aLittleInwardPose = PoseFactory.of(closestPtOnLoadingZone).plus(inwards.toPose());

        Interpolator closestTangent = CommonKt.fastInterpolation(robotPose, aLittleInwardPose);

        // Convert Pedro Pathing Poses
        Pose startPose = new Pose(robotPose.x, robotPose.y, robotPose.heading);
        Pose endPose = new Pose(aLittleInwardPose.x, aLittleInwardPose.y, aLittleInwardPose.heading);

        // Inflate obstacles by the robot's physical boundaries (half-diagonal dimension acting as radius safety buffer)
        double safetyRadius = Math.hypot(botLength, botWidth) / 2.0 + 1.5;
        BufferParameters bufParams = new BufferParameters();
        bufParams.setJoinStyle(BufferParameters.JOIN_MITRE);
        Geometry forbiddenSpace = BufferOp.bufferOp(obstacles, safetyRadius, bufParams);

        // Check if direct line-of-sight intersects forbidden space boundaries
        Coordinate[] lineCoords = new Coordinate[]{ robotPose.toCoordinate(), aLittleInwardPose.toCoordinate() };
        Geometry travelLine = geometryFactory.createLineString(lineCoords);

        if (travelLine.intersects(forbiddenSpace)) {
            // Pull the centroid of the intersection space to compute a safe bypass control point
            Geometry intersection = travelLine.intersection(forbiddenSpace);
            Coordinate center = intersection.getCentroid().getCoordinate();

            // Push the control point outward normal to the obstruction direction
            double pushAmp = safetyRadius * 1.3;
            double normalTheta = theta + Math.PI / 2.0;
            Pose controlPose = new Pose(center.x + pushAmp * Math.cos(normalTheta), center.y + pushAmp * Math.sin(normalTheta));

            // Return a curved Bezier Path to dodge the environment hazards safely
            return Paths.curve(startPose, controlPose, endPose).heading(closestTangent);
        }

        // Return a straight line path if the line-of-sight trajectory remains unobstructed
        return Paths.line(startPose, endPose).heading(closestTangent);
    }
     */
}
