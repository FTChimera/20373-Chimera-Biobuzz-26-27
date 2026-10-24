package org.firstinspires.ftc.teamcode.util.poses;

import static org.firstinspires.ftc.teamcode.util.math.MathUtils.DEG_TO_RAD;

import com.pedropathing.math.Pose;

import org.locationtech.jts.geom.Coordinate;

public class PoseFactory {
    //public static PoseFactory INSTANCE = new PoseFactory();

    public static Pose2d deg(double x, double y, double heading) {
        return new Pose2d(x, y, heading*DEG_TO_RAD);
    }
    public static Pose2d deg(double heading) {
        return new Pose2d(0, 0, heading*DEG_TO_RAD);
    }

    /* Redundant
    public static Pose2d rad(double x, double y, double heading) {
        return new Pose2d(x, y, heading);
    }
     */

    public static Pose2d zero() {
        return Pose2d.ZERO;
    }

    public static Pose2d of(double x, double y, double heading) {
        return new Pose2d(x, y, heading);
    }

    public static Pose2d of(double heading) {
        return new Pose2d(0, 0, heading);
    }

    public static Pose2d of(double x, double y) {
        return new Pose2d(x, y);
    }

    public static Pose2d of(Pose pose) {
        return new Pose2d(pose.x(), pose.y(), pose.heading());
    }

    public static Pose2d of(Coordinate coordinate) {
        return new Pose2d(coordinate.getX(), coordinate.getY());
    }

    @FunctionalInterface
    public interface Operation {
        Pose2d apply(Pose2d input);
        static Operation identity() {
            return input -> input;
        }
        static Pose2d convertDegToRad(Pose2d pose) {
            return pose.withHeading(pose.heading()*DEG_TO_RAD);
        }
        static Operation degToRad() {
            return Operation::convertDegToRad;
        }
        default Operation compose(Operation operation) {
            return input -> operation.apply(this.apply(input));
        }
    }

    Operation operation;
    public PoseFactory(Operation operation) {
        this.operation = operation;
    }

    public Pose2d create(double x, double y, double heading) {
        return operation.apply(of(x, y, heading));
    }

    public Pose2d create(Pose pose) {
        return operation.apply(of(pose));
    }

    public Pose2d create(double x, double y) {
        return operation.apply(of(x, y));
    }
    public Pose2d create(double heading) {
        return operation.apply(of(heading));
    }

    public Pose2d create(Coordinate coordinate) {
        return operation.apply(of(coordinate));
    }

}
