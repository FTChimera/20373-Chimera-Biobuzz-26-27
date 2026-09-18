package org.firstinspires.ftc.teamcode.library.math;

import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.geometry.CoordinateSystem;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;
import java.util.Objects;

public final class MathUtils {
    public static final Companion Companion = new Companion();
    public static final MathUnits units = new MathUnits();

    public static final double PI = Math.PI;
    public static final double TAU = 2 * PI;
    public static final double E = Math.E;
    public static final double DEG_TO_RAD = PI / 180.0;
    public static final double EPSILON = 1e-6;

    private static final double MILLIMETERS_PER_INCH = 25.4;
    private static final double CENTIMETERS_PER_INCH = 2.54;
    private static final double INCHES_PER_FOOT = 12.0;
    private static final double INCHES_PER_METER = 39.37007874015748;

    public static final Polynomial SMOOTH_STEP_FUNCTION =
            new Polynomial(new double[]{0.0, 0.0, 3.0, -2.0});
    public static final Polynomial SMOOTHER_STEP_FUNCTION =
            new Polynomial(new double[]{0.0, 0.0, 0.0, 10.0, -15.0, 6.0});

    private MathUtils() {
    }

    public static double distance(double x1, double y1, double x2, double y2) {
        return Math.hypot(x2 - x1, y2 - y1);
    }

    public static double headingBetween(double x1, double y1, double x2, double y2) {
        return Math.atan2(y2 - y1, x2 - x1);
    }

    public static double headingDelta(double fromHeading, double toHeading) {
        return wrapAngle(toHeading - fromHeading);
    }

    public static Pose normalizeVector(double x, double y) {
        double magnitude = Math.hypot(x, y);
        if (magnitude < EPSILON) {
            return new Pose(1.0, 0.0);
        }
        return new Pose(x / magnitude, y / magnitude);
    }

    public static Pose normalizedDirection(Pose start, Pose end) {
        Objects.requireNonNull(start, "start");
        Objects.requireNonNull(end, "end");
        return normalizeVector(end.getX() - start.getX(), end.getY() - start.getY());
    }

    public static Pose addScaled(Pose pose, Pose direction, double scale) {
        Objects.requireNonNull(pose, "pose");
        Objects.requireNonNull(direction, "direction");
        return new Pose(
                pose.getX() + direction.getX() * scale,
                pose.getY() + direction.getY() * scale
        );
    }

    public static Pose[] smoothTangents(List<Pose> waypoints) {
        Objects.requireNonNull(waypoints, "waypoints");
        requireAtLeastTwoWaypoints(waypoints);

        Pose[] tangents = new Pose[waypoints.size()];
        for (int index = 0; index < waypoints.size(); index++) {
            if (index == 0) {
                tangents[index] = normalizedDirection(waypoints.get(0), waypoints.get(1));
            } else if (index == waypoints.size() - 1) {
                tangents[index] = normalizedDirection(
                        waypoints.get(waypoints.size() - 2), waypoints.get(waypoints.size() - 1));
            } else {
                Pose incoming = normalizedDirection(waypoints.get(index - 1), waypoints.get(index));
                Pose outgoing = normalizedDirection(waypoints.get(index), waypoints.get(index + 1));
                tangents[index] = normalizeVector(
                        incoming.getX() + outgoing.getX(), incoming.getY() + outgoing.getY());
            }
        }
        return tangents;
    }

    public static double[] adaptiveHandleLengths(List<Pose> waypoints, double maxHandleFraction) {
        Objects.requireNonNull(waypoints, "waypoints");
        requireAtLeastTwoWaypoints(waypoints);

        double[] handleLengths = new double[waypoints.size()];
        for (int index = 0; index < waypoints.size(); index++) {
            if (index == 0) {
                handleLengths[index] = waypoints.get(0).distanceFrom(waypoints.get(1))
                        * maxHandleFraction;
            } else if (index == waypoints.size() - 1) {
                handleLengths[index] = waypoints.get(waypoints.size() - 2)
                        .distanceFrom(waypoints.get(waypoints.size() - 1)) * maxHandleFraction;
            } else {
                Pose previous = waypoints.get(index - 1);
                Pose current = waypoints.get(index);
                Pose next = waypoints.get(index + 1);

                double previousDistance = previous.distanceFrom(current);
                double nextDistance = current.distanceFrom(next);
                Pose previousDirection = normalizedDirection(previous, current);
                Pose nextDirection = normalizedDirection(current, next);
                double dotProduct = clamp(
                        previousDirection.getX() * nextDirection.getX()
                                + previousDirection.getY() * nextDirection.getY(),
                        -1.0,
                        1.0
                );
                double turnScale = Math.sqrt(Math.max(0.0, (1.0 + dotProduct) / 2.0));

                handleLengths[index] = Math.min(previousDistance, nextDistance)
                        * maxHandleFraction * turnScale;
            }
        }
        return handleLengths;
    }

    public static double maxTurnAngle(List<Pose> waypoints) {
        Objects.requireNonNull(waypoints, "waypoints");
        if (waypoints.size() < 3) {
            return 0.0;
        }

        double maxTurn = 0.0;
        for (int index = 1; index < waypoints.size() - 1; index++) {
            Pose previousDirection = normalizedDirection(waypoints.get(index - 1), waypoints.get(index));
            Pose nextDirection = normalizedDirection(waypoints.get(index), waypoints.get(index + 1));
            double dotProduct = clamp(
                    previousDirection.getX() * nextDirection.getX()
                            + previousDirection.getY() * nextDirection.getY(),
                    -1.0,
                    1.0
            );
            maxTurn = Math.max(maxTurn, Math.acos(dotProduct));
        }
        return maxTurn;
    }

    public static double maxSpacingRatio(List<Pose> waypoints) {
        Objects.requireNonNull(waypoints, "waypoints");
        if (waypoints.size() < 3) {
            return 1.0;
        }

        double minDistance = Double.POSITIVE_INFINITY;
        double maxDistance = 0.0;
        for (int index = 0; index < waypoints.size() - 1; index++) {
            double distance = waypoints.get(index).distanceFrom(waypoints.get(index + 1));
            if (distance < EPSILON) {
                continue;
            }
            minDistance = Math.min(minDistance, distance);
            maxDistance = Math.max(maxDistance, distance);
        }

        if (!Double.isFinite(minDistance) || minDistance < EPSILON) {
            return Double.POSITIVE_INFINITY;
        }
        return maxDistance / minDistance;
    }

    public static Pose offsetPoseInRobotFrame(
            Pose robotPose,
            double forwardDistance,
            double lateralOffset,
            CoordinateSystem coordinateSystem
    ) {
        Objects.requireNonNull(robotPose, "robotPose");
        Objects.requireNonNull(coordinateSystem, "coordinateSystem");
        Pose convertedRobotPose = coordinateSystem.convertFromPedro(robotPose);
        double heading = convertedRobotPose.getHeading();
        double fieldX = convertedRobotPose.getX() + forwardDistance * Math.cos(heading)
                - lateralOffset * Math.sin(heading);
        double fieldY = convertedRobotPose.getY() + forwardDistance * Math.sin(heading)
                + lateralOffset * Math.cos(heading);
        return coordinateSystem.convertToPedro(new Pose(fieldX, fieldY, heading, coordinateSystem));
    }

    public static double wrapAngle(double angle) {
        return units.normalizeHeading(angle, AngleUnit.RADIANS, HeadingNormSettings.CENTERED_AT_ZERO);
    }

    public static Polynomial access$getSMOOTH_STEP_FUNCTION$cp() {
        return SMOOTH_STEP_FUNCTION;
    }

    public static Polynomial access$getSMOOTHER_STEP_FUNCTION$cp() {
        return SMOOTHER_STEP_FUNCTION;
    }

    private static void requireAtLeastTwoWaypoints(List<Pose> waypoints) {
        if (waypoints.size() < 2) {
            throw new IllegalArgumentException("Need at least two waypoints");
        }
    }

    private static double clamp(double value, double minimum, double maximum) {
        return Math.max(minimum, Math.min(maximum, value));
    }

    public enum HeadingNormSettings {
        CENTERED_AT_ZERO,
        POSITIVE,
        NEGATIVE
    }

    public enum DistanceUnit {
        MILLIMETERS,
        CENTIMETERS,
        METERS,
        INCHES,
        FEET
    }

    public enum AngleUnit {
        DEGREES,
        RADIANS
    }

    public static final class Companion {
        private Companion() {
        }

        public Polynomial getSMOOTH_STEP_FUNCTION() {
            return SMOOTH_STEP_FUNCTION;
        }

        public Polynomial getSMOOTHER_STEP_FUNCTION() {
            return SMOOTHER_STEP_FUNCTION;
        }

        public double distance(double x1, double y1, double x2, double y2) {
            return MathUtils.distance(x1, y1, x2, y2);
        }

        public double headingBetween(double x1, double y1, double x2, double y2) {
            return MathUtils.headingBetween(x1, y1, x2, y2);
        }

        public double headingDelta(double fromHeading, double toHeading) {
            return MathUtils.headingDelta(fromHeading, toHeading);
        }

        public Pose normalizeVector(double x, double y) {
            return MathUtils.normalizeVector(x, y);
        }

        public Pose normalizedDirection(Pose start, Pose end) {
            return MathUtils.normalizedDirection(start, end);
        }

        public Pose addScaled(Pose pose, Pose direction, double scale) {
            return MathUtils.addScaled(pose, direction, scale);
        }

        public Pose[] smoothTangents(List<Pose> waypoints) {
            return MathUtils.smoothTangents(waypoints);
        }

        public double[] adaptiveHandleLengths(List<Pose> waypoints, double maxHandleFraction) {
            return MathUtils.adaptiveHandleLengths(waypoints, maxHandleFraction);
        }

        public double maxTurnAngle(List<Pose> waypoints) {
            return MathUtils.maxTurnAngle(waypoints);
        }

        public double maxSpacingRatio(List<Pose> waypoints) {
            return MathUtils.maxSpacingRatio(waypoints);
        }

        public Pose offsetPoseInRobotFrame(
                Pose robotPose,
                double forwardDistance,
                double lateralOffset,
                CoordinateSystem coordinateSystem
        ) {
            return MathUtils.offsetPoseInRobotFrame(
                    robotPose, forwardDistance, lateralOffset, coordinateSystem);
        }

        public double wrapAngle(double angle) {
            return MathUtils.wrapAngle(angle);
        }
    }

    public static final class MathUnits {
        public double toMillimeters(double num, DistanceUnit unit) {
            Objects.requireNonNull(unit, "unit");
            switch (unit) {
                case MILLIMETERS:
                    return num;
                case CENTIMETERS:
                    return num * 10.0;
                case METERS:
                    return num * 1000.0;
                case INCHES:
                    return num * MILLIMETERS_PER_INCH;
                case FEET:
                    return num * INCHES_PER_FOOT * MILLIMETERS_PER_INCH;
                default:
                    throw new IllegalArgumentException("Unknown distance unit: " + unit);
            }
        }

        public double toCentimeters(double num, DistanceUnit unit) {
            return toMillimeters(num, unit) / 10.0;
        }

        public double toMeters(double num, DistanceUnit unit) {
            return toMillimeters(num, unit) / 1000.0;
        }

        public double toInches(double num, DistanceUnit unit) {
            Objects.requireNonNull(unit, "unit");
            switch (unit) {
                case MILLIMETERS:
                    return num / MILLIMETERS_PER_INCH;
                case CENTIMETERS:
                    return num / CENTIMETERS_PER_INCH;
                case METERS:
                    return num * INCHES_PER_METER;
                case INCHES:
                    return num;
                case FEET:
                    return num * INCHES_PER_FOOT;
                default:
                    throw new IllegalArgumentException("Unknown distance unit: " + unit);
            }
        }

        public double toFeet(double num, DistanceUnit unit) {
            return toInches(num, unit) / INCHES_PER_FOOT;
        }

        public double toDegrees(double num, AngleUnit unit) {
            Objects.requireNonNull(unit, "unit");
            return unit == AngleUnit.DEGREES ? num : num / DEG_TO_RAD;
        }

        public double toRadians(double num, AngleUnit unit) {
            Objects.requireNonNull(unit, "unit");
            return unit == AngleUnit.DEGREES ? num * DEG_TO_RAD : num;
        }

        public double toDegrees(double num) {
            return toDegrees(num, AngleUnit.RADIANS);
        }

        public double toRadians(double num) {
            return toRadians(num, AngleUnit.DEGREES);
        }

        public Pose toPedroPathing(Pose2D ftcPose) {
            Objects.requireNonNull(ftcPose, "ftcPose");
            return new Pose(
                    ftcPose.getX(org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.INCH)
                            + 72,
                    ftcPose.getY(org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.INCH)
                            + 72,
                    ftcPose.getHeading(org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.RADIANS),
                    FTCCoordinates.INSTANCE
            ).getAsCoordinateSystem(PedroCoordinates.INSTANCE);
        }

        public Pose toPedroPathing(Pose3D ftcPose) {
            Objects.requireNonNull(ftcPose, "ftcPose");
            return toPedroPathing(new Pose2D(
                    ftcPose.getPosition().unit,
                    ftcPose.getPosition().x,
                    ftcPose.getPosition().y,
                    org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.RADIANS,
                    ftcPose.getOrientation().getYaw(
                            org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.RADIANS)
            ));
        }

        public Pose2D toFTCPose2D(Pose pedroPose) {
            Objects.requireNonNull(pedroPose, "pedroPose");
            Pose convertedPose = pedroPose.getAsCoordinateSystem(FTCCoordinates.INSTANCE);
            return new Pose2D(
                    org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.INCH,
                    convertedPose.getX(),
                    convertedPose.getY(),
                    org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.RADIANS,
                    convertedPose.getHeading()
            );
        }

        public double normalizeHeading(
                double heading,
                AngleUnit unit,
                HeadingNormSettings settings
        ) {
            Objects.requireNonNull(unit, "unit");
            Objects.requireNonNull(settings, "settings");
            double fullTurn = unit == AngleUnit.DEGREES ? 360.0 : TAU;
            double halfTurn = fullTurn / 2.0;
            double positiveWrapped = ((heading % fullTurn) + fullTurn) % fullTurn;

            switch (settings) {
                case CENTERED_AT_ZERO:
                    return positiveWrapped > halfTurn ? positiveWrapped - fullTurn : positiveWrapped;
                case POSITIVE:
                    return positiveWrapped;
                case NEGATIVE:
                    return positiveWrapped == 0.0 ? 0.0 : positiveWrapped - fullTurn;
                default:
                    throw new IllegalArgumentException(
                            "Unknown heading normalization setting: " + settings);
            }
        }
    }

    public static final class MathFunc {
        public Polynomial smoothFunc() {
            return SMOOTHER_STEP_FUNCTION;
        }

        public double evalSmoothF(double t) {
            return smoothFunc().getY(t);
        }

        public double evalSmoothFInterp(double t, double h0, double h1) {
            return h0 + (h1 - h0) * evalSmoothF(t);
        }
    }
}
