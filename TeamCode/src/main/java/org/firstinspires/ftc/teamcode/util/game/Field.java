package org.firstinspires.ftc.teamcode.util.game;

import androidx.annotation.NonNull;

import com.pedropathing.api.Paths;
import com.rmsftc.utils.math.Pose;
import com.pedropathing.math.Vector2D;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.interpolator.Interpolator;

import org.firstinspires.ftc.teamcode.util.poses.PoseFactory;
import org.firstinspires.ftc.teamcode.planners.CommonKt;
import org.firstinspires.ftc.teamcode.planners.FieldCoordinates;
import org.firstinspires.ftc.teamcode.planners.FieldPoses;
import com.rmsftc.utils.AllianceColor;
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
                    return FieldPoses.redTippedHiveGoal();
                } else {
                    return FieldPoses.redUntippedHiveGoal();
                }
            }
            if (allianceColor.isBlue()) {
                if (this == TIPPED) {
                    return FieldPoses.blueTippedHiveGoal();
                } else {
                    return FieldPoses.blueUntippedHiveGoal();
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

    private static final double TRANSIT_CLEARANCE_INCHES = 1.0;

    // Defined loading zones using 24x24 inch floor tiles corners on standard 144x144 field layout
    public static final Coordinate[] RED_LOADING_ZONE = FieldCoordinates.redLoadingZone();
    public static final Coordinate[] BLUE_LOADING_ZONE = FieldCoordinates.blueLoadingZone();

    public double botWidth, botLength;

    public Field(AllianceColor color, double botWidth, double botLength) {
        hiveState = HiveState.UNTIPPED;
        this.botLength = botLength;
        this.botWidth = botWidth;

        loadingZone = color.isRed() ?
                geometryFactory.createPolygon(RED_LOADING_ZONE) :
                geometryFactory.createPolygon(BLUE_LOADING_ZONE);

        // Model static obstacles on the field (e.g., Hive structure bases modeled as rectangles)
        Polygon hiveBase1 = geometryFactory.createPolygon(FieldCoordinates.westHiveBase());
        Polygon hiveBase2 = geometryFactory.createPolygon(FieldCoordinates.eastHiveBase());
        obstacles = hiveBase1.union(hiveBase2);
        // FLAGGED CHANGE: FLOWER footprints, flush against the perimeter walls.
        // Kept separate from `obstacles`: the safety-radius buffer applies to the
        // hive only, so flowers join the forbidden space RAW (see the forbiddenSpace
        // line in generatePathToLoadingZoneShortest).
        Polygon flowerWest = geometryFactory.createPolygon(FieldCoordinates.westFlower());
        Polygon flowerNorth = geometryFactory.createPolygon(FieldCoordinates.northFlower());
        Polygon flowerEast = geometryFactory.createPolygon(FieldCoordinates.eastFlower());
        Polygon flowerSouth = geometryFactory.createPolygon(FieldCoordinates.southFlower());
        flowerObstacles = flowerWest.union(flowerNorth).union(flowerEast).union(flowerSouth);
    }

    public void setHiveState(HiveState set) {
        this.hiveState = set;
    }

    public HiveState getHiveState() {
        return hiveState;
    }

    public double getClosestLoadingZoneDistanceToRobot(Pose robotPose) {
        return loadingZone.distance(geometryFactory.createPoint(new Coordinate(robotPose.x(), robotPose.y())));
    }

    public boolean isRobotOverlappingLoadingZone(Pose robotPose) {
        return loadingZone.intersects(createRobotGeom(robotPose));
    }

    public Geometry createRobotGeom(Pose robotPose) {
        double halfLength = botLength / 2;
        double halfWidth = botWidth / 2;
        return geometryFactory.createPolygon(new Coordinate[]{
                new Coordinate(robotPose.x() - halfLength, robotPose.y() - halfWidth),
                new Coordinate(robotPose.x() + halfLength, robotPose.y() - halfWidth),
                new Coordinate(robotPose.x() + halfLength, robotPose.y() + halfWidth),
                new Coordinate(robotPose.x() - halfLength, robotPose.y() + halfWidth),
                new Coordinate(robotPose.x() - halfLength, robotPose.y() - halfWidth)
        });
    }

    public Path generatePathToLoadingZoneShortest(Pose robotPose, double innerVal) {
        if (isRobotOverlappingLoadingZone(robotPose)) {
            throw new NoRouteException("Robot already overlaps the loading zone; no park path is required");
        }
        // Park target: aim the robot CENTER at a point inside the zone. The
        // old target (closest body point moved innerVal past the boundary
        // along the approach direction) left most of the robot outside the
        // zone on diagonal approaches, because the body trails back along
        // the approach diagonal. Now: from the closest boundary point, move
        // toward the zone centroid by innerVal plus the body's extent along
        // that direction, so the leading face ends innerVal inside the
        // boundary; clamp the center so the body never targets through a
        // field wall.
        Coordinate closestPtOnLoadingZone = DistanceOp.nearestPoints(
                loadingZone, geometryFactory.createPoint(new Coordinate(robotPose.x(), robotPose.y())))[0];
        Coordinate centroid = loadingZone.getCentroid().getCoordinate();
        double inwardX = centroid.x - closestPtOnLoadingZone.x;
        double inwardY = centroid.y - closestPtOnLoadingZone.y;
        double inwardLen = Math.hypot(inwardX, inwardY);
        if (inwardLen < 1e-9) {
            // Closest point is the centroid (robot center inside the zone):
            // fall back to the direction from the robot to that point.
            inwardX = closestPtOnLoadingZone.x - robotPose.x();
            inwardY = closestPtOnLoadingZone.y - robotPose.y();
            inwardLen = Math.hypot(inwardX, inwardY);
        }
        double targetX = closestPtOnLoadingZone.x;
        double targetY = closestPtOnLoadingZone.y;
        if (inwardLen >= 1e-9) {
            double dirX = inwardX / inwardLen;
            double dirY = inwardY / inwardLen;
            // Extent of the (axis-aligned) robot rectangle along the inward
            // direction: how far the leading face is from the center.
            double halfExtent = (botLength / 2.0) * Math.abs(dirX) + (botWidth / 2.0) * Math.abs(dirY);
            double dist = Math.min(innerVal + halfExtent, inwardLen);
            targetX += dirX * dist;
            targetY += dirY * dist;
        }
        // Keep the whole body on the field: the center stays half a robot
        // from each wall (the red zone is against the west/north walls).
        targetX = Math.max(botLength / 2.0, Math.min(144.0 - botLength / 2.0, targetX));
        targetY = Math.max(botWidth / 2.0, Math.min(144.0 - botWidth / 2.0, targetY));
        Pose aLittleInwardPose = PoseFactory.of(targetX, targetY, 0);

        // Convert Pedro Pathing Poses
        Pose startPose = PoseFactory.of(robotPose.x(), robotPose.y(), robotPose.heading());
        Pose endPose = PoseFactory.of(aLittleInwardPose.x(), aLittleInwardPose.y(), aLittleInwardPose.heading());

        endPose.setHeading(roundToCardinal(Math.atan2(endPose.y() - startPose.y(), endPose.x() - startPose.x())));
        return generateTransitPath(startPose, endPose);
    }

    /** Collision-checked transit with clearance for the body at every heading. */
    public Path generateTransitPath(Pose startPose, Pose endPose) {
        return transitSegments(transitWaypoints(startPose, endPose));
    }

    private List<Pose> transitWaypoints(Pose startPose, Pose endPose) {
        double radius = Math.hypot(botLength, botWidth) / 2.0 + TRANSIT_CLEARANCE_INCHES;
        BufferParameters parameters = new BufferParameters();
        parameters.setJoinStyle(BufferParameters.JOIN_MITRE);
        Geometry forbidden = BufferOp.bufferOp(obstacles, radius, parameters)
                .union(flowerObstacles.buffer(radius));
        Pose insetStart = insetForTurning(startPose, radius);
        Pose insetEnd = insetForTurning(endPose, radius);
        List<Pose> poses = pathfinder.findPath(insetStart, insetEnd, forbidden);
        if (poses.size() < 2) {
            throw new NoRouteException("No body-clear transit route from " + startPose + " to " + endPose);
        }
        if (startPose.distance(insetStart) > 1e-6) poses.add(0, Pose.fromPedro(startPose));
        if (endPose.distance(insetEnd) > 1e-6) poses.add(Pose.fromPedro(endPose));
        return poses;
    }

    private static Pose insetForTurning(Pose pose, double radius) {
        return new Pose(Math.max(radius, Math.min(144 - radius, pose.x())),
                Math.max(radius, Math.min(144 - radius, pose.y())), pose.heading());
    }

    private static Path transitSegments(List<Pose> poses) {
        Path[] segments = new Path[poses.size() - 1];
        for (int i = 0; i < segments.length; i++) {
            Pose from = poses.get(i);
            Pose to = poses.get(i + 1);
            // Endpoints near walls retain their heading; rotation happens at
            // the inset waypoints, where the full body diagonal fits.
            segments[i] = Paths.line(from, to).linear(from, to);
        }
        // A smoothing spline can cut back through the obstacles avoided by Dijkstra.
        return Paths.path(segments);
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
