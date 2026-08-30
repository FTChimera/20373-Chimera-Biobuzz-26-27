package org.firstinspires.ftc.teamcode.library.game;


import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.library.AllianceColor;
import org.locationtech.jts.geom.Coordinate;
import org.locationtech.jts.geom.Geometry;
import org.locationtech.jts.geom.GeometryFactory;
import org.locationtech.jts.geom.Polygon;
import org.locationtech.jts.util.GeometricShapeFactory;

import java.util.Arrays;
import java.util.Comparator;
import java.util.Objects;

public class Field {
    private static final int CIRCLE_POINT_COUNT = 50;

    public static final class Factory {
        public static final GeometryFactory INSTANCE = new GeometryFactory();
    }
    public static Polygon createPolygon(Pose... pts) {
        Objects.requireNonNull(pts, "pts");
        Coordinate[] coords = new Coordinate[pts.length];

        for (int i = 0; i < pts.length; i++) {
            Pose pose = Objects.requireNonNull(pts[i], "pts[" + i + "]");
            coords[i] = new Coordinate(pose.getX(), pose.getY());
        }

        return createOrderedPolygon(coords);
    }
    public static Polygon createOrderedPolygon(Coordinate[] unorderedCoords) {
        if (unorderedCoords == null || unorderedCoords.length < 3) {
            throw new IllegalArgumentException("At least 3 coordinates are required.");
        }
        for (int i = 0; i < unorderedCoords.length; i++) {
            Coordinate coordinate = Objects.requireNonNull(unorderedCoords[i], "unorderedCoords[" + i + "]");
            validateFinite("coordinate[" + i + "].x", coordinate.x);
            validateFinite("coordinate[" + i + "].y", coordinate.y);
        }

        // 1. Calculate the center (mean) point of all coordinates
        double sumX = 0;
        double sumY = 0;
        for (Coordinate c : unorderedCoords) {
            sumX += c.x;
            sumY += c.y;
        }
        final double centerX = sumX / unorderedCoords.length;
        final double centerY = sumY / unorderedCoords.length;

        // 2. Sort unique coordinates based on their angle from the center
        Coordinate[] sorted = Arrays.stream(unorderedCoords)
                .distinct() // Remove duplicates to ensure clean calculation
                .sorted(Comparator.comparingDouble(c -> Math.atan2(c.y - centerY, c.x - centerX)))
                .toArray(Coordinate[]::new);
        if (sorted.length < 3) {
            throw new IllegalArgumentException("At least 3 distinct coordinates are required.");
        }

        // 3. Create a new array that is 1 item larger to close the loop
        Coordinate[] closedCoords = new Coordinate[sorted.length + 1];
        System.arraycopy(sorted, 0, closedCoords, 0, sorted.length);
        closedCoords[closedCoords.length - 1] = new Coordinate(sorted[0]); // Duplicate first item at the end

        Polygon polygon = Factory.INSTANCE.createPolygon(closedCoords);
        if (!polygon.isValid() || polygon.getArea() <= 0) {
            throw new IllegalArgumentException("Coordinates must form a non-degenerate polygon.");
        }
        return polygon;
    }
    public static Polygon createCircle(Pose center, double radius) {
        Objects.requireNonNull(center, "center");
        validateFinite("center.x", center.getX());
        validateFinite("center.y", center.getY());
        validateFinite("radius", radius);
        if (radius < 0) {
            throw new IllegalArgumentException("radius cannot be negative.");
        }
        GeometricShapeFactory sf = new GeometricShapeFactory(Factory.INSTANCE);
        sf.setCentre(new Coordinate(center.getX(), center.getY()));
        sf.setSize(radius * 2); // set diameter
        sf.setNumPoints(CIRCLE_POINT_COUNT);
        return sf.createCircle();
    }

    private static Coordinate toCoordinate(Pose pose) {
        return new Coordinate(pose.getX(), pose.getY());
    }

    private static void validateFinite(String name, double value) {
        if (!Double.isFinite(value)) {
            throw new IllegalArgumentException(name + " must be finite.");
        }
    }

    private static double getRobotWidth() {
        return Constants.currentBot.BOT_WIDTH;
    }

    private static double getRobotLength() {
        return Constants.currentBot.BOT_LENGTH;
    }

    private static Coordinate createOffsetCoordinate(Pose center, double forwardOffset, double leftOffset) {
        double heading = center.getHeading();
        double cos = Math.cos(heading);
        double sin = Math.sin(heading);
        double x = center.getX() + (forwardOffset * cos) - (leftOffset * sin);
        double y = center.getY() + (forwardOffset * sin) + (leftOffset * cos);
        return new Coordinate(x, y);
    }

    private static Polygon createRectangle(Pose center, double robotWidth, double robotLength) {
        double halfWidth = robotWidth / 2.0;
        double halfLength = robotLength / 2.0;
        return Factory.INSTANCE.createPolygon(new Coordinate[]{
                createOffsetCoordinate(center, halfLength, halfWidth),
                createOffsetCoordinate(center, halfLength, -halfWidth),
                createOffsetCoordinate(center, -halfLength, -halfWidth),
                createOffsetCoordinate(center, -halfLength, halfWidth),
                createOffsetCoordinate(center, halfLength, halfWidth)
        });
    }

    private static Geometry createRobotLine(Pose center, double robotWidth, double robotLength) {
        if (robotLength == 0) {
            return Factory.INSTANCE.createLineString(new Coordinate[]{
                    createOffsetCoordinate(center, 0, robotWidth / 2.0),
                    createOffsetCoordinate(center, 0, -robotWidth / 2.0)
            });
        }
        return Factory.INSTANCE.createLineString(new Coordinate[]{
                createOffsetCoordinate(center, robotLength / 2.0, 0),
                createOffsetCoordinate(center, -robotLength / 2.0, 0)
        });
    }

    private static void validateRobotMotion(Pose position, Vector velocity, double deltaTime, double robotWidth, double robotLength) {
        Objects.requireNonNull(position, "position cannot be null.");
        Objects.requireNonNull(velocity, "velocity cannot be null.");

        validateFinite("position.x", position.getX());
        validateFinite("position.y", position.getY());
        validateFinite("position.heading", position.getHeading());
        validateFinite("velocity.x", velocity.getXComponent());
        validateFinite("velocity.y", velocity.getYComponent());
        validateFinite("deltaTime", deltaTime);
        validateFinite("robotWidth", robotWidth);
        validateFinite("robotLength", robotLength);

        if (deltaTime < 0) {
            throw new IllegalArgumentException("deltaTime cannot be negative.");
        }
        if (robotWidth < 0) {
            throw new IllegalArgumentException("robotWidth cannot be negative.");
        }
        if (robotLength < 0) {
            throw new IllegalArgumentException("robotLength cannot be negative.");
        }
    }

    private static Geometry createRobotFootprint(Pose center, double robotWidth, double robotLength) {
        if (robotWidth == 0 && robotLength == 0) {
            return Factory.INSTANCE.createPoint(toCoordinate(center));
        }
        if (robotWidth == 0 || robotLength == 0) {
            return createRobotLine(center, robotWidth, robotLength);
        }
        return createRectangle(center, robotWidth, robotLength);
    }

    private static Pose projectPosition(Pose position, Vector velocity, double deltaTime) {
        return new Pose(
                position.getX() + (velocity.getXComponent() * deltaTime),
                position.getY() + (velocity.getYComponent() * deltaTime),
                position.getHeading(),
                position.getCoordinateSystem()
        );
    }

    private static Geometry createRobotSweep(Pose position, Vector velocity, double deltaTime, double robotWidth, double robotLength) {
        Pose projectedPosition = projectPosition(position, velocity, deltaTime);
        Coordinate start = toCoordinate(position);
        Coordinate end = toCoordinate(projectedPosition);

        if (start.equals2D(end)) {
            return createRobotFootprint(position, robotWidth, robotLength);
        }
        return createRobotFootprint(position, robotWidth, robotLength)
                .union(createRobotFootprint(projectedPosition, robotWidth, robotLength))
                .convexHull();
    }

    private static ContactState getContactState(Polygon polygon, Pose position, Vector velocity, double deltaTime, double robotWidth, double robotLength) {
        validateRobotMotion(position, velocity, deltaTime, robotWidth, robotLength);

        Geometry robotSweep = createRobotSweep(position, velocity, deltaTime, robotWidth, robotLength);
        if (polygon.covers(robotSweep)) {
            return ContactState.CONTAINED;
        }
        if (polygon.intersects(robotSweep)) {
            return ContactState.PARTIAL;
        }
        return ContactState.OUTSIDE;
    }

    public enum Shapes {
        SOME_TRIANGLE(createSome_Triangle()),
        SOME_CIRCLE(createSome_Circle());

        Shapes(Polygon polygon) {
            shape = polygon;
        }
        private final Polygon shape;
        private static Polygon createSome_Triangle() {
            return createPolygon(
                    new Pose(96, 96),
                    new Pose(96, -96),
                    new Pose(0,  0)
            );
        }
        private static Polygon createSome_Circle() {
            return createCircle(
                    new Pose(30, 40),
                    10
            );
        }
        public Polygon getShape() {
            return shape;
        }

        public ContactState getContactState(Pose position, Vector velocity, double deltaTime) {
            return getContactState(position, velocity, deltaTime, getRobotWidth(), getRobotLength());
        }

        public ContactState getContactState(Pose position, Pose velocity, double deltaTime) {
            return getContactState(position, new Vector(velocity), deltaTime, getRobotWidth(), getRobotLength());
        }

        private ContactState getContactState(Pose position, Vector velocity, double deltaTime, double robotWidth, double robotLength) {
            return Field.getContactState(shape, position, velocity, deltaTime, robotWidth, robotLength);
        }

        ContactState getContactState(Pose position, Pose velocity, double deltaTime, double robotWidth, double robotLength) {
            return getContactState(position, new Vector(velocity), deltaTime, robotWidth, robotLength);
        }

    }
    public enum Boundaries {
        AUTO_MID_LINE_BLUE(createAuto_Mid_Line(AllianceColor.BLUE)),
        AUTO_MID_LINE_RED(createAuto_Mid_Line(AllianceColor.RED));

        public final Polygon polygon;
        Boundaries(Polygon poses) {
            polygon = poses;
        }

        private static Polygon createAuto_Mid_Line(AllianceColor color) {
            return createPolygon(
                    new Pose(72, 144),
                    new Pose(72, 0),
                    new Pose(color==AllianceColor.RED ? 144 : 0, 0),
                    new Pose(color==AllianceColor.RED ? 144 : 0, 144)
            );
        }

        public ContactState getContactState(Pose position, Vector velocity, double deltaTime) {
            return getContactState(position, velocity, deltaTime, getRobotWidth(), getRobotLength());
        }

        public ContactState getContactState(Pose position, Pose velocity, double deltaTime) {
            return getContactState(position, new Vector(velocity), deltaTime, getRobotWidth(), getRobotLength());
        }

        ContactState getContactState(Pose position, Vector velocity, double deltaTime, double robotWidth, double robotLength) {
            return Field.getContactState(polygon, position, velocity, deltaTime, robotWidth, robotLength);
        }

        private ContactState getContactState(Pose position, Pose velocity, double deltaTime, double robotWidth, double robotLength) {
            return getContactState(position, new Vector(velocity), deltaTime, robotWidth, robotLength);
        }

    }

    public enum ContactState {
        OUTSIDE,
        PARTIAL,
        CONTAINED
    }
}
