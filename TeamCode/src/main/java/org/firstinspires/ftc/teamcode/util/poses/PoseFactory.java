package org.firstinspires.ftc.teamcode.util.poses;

import static org.firstinspires.ftc.teamcode.util.math.MathUtils.DEG_TO_RAD;

import com.rmsftc.utils.math.Pose;

import org.locationtech.jts.geom.Coordinate;

public class PoseFactory {
    //public static PoseFactory INSTANCE = new PoseFactory();

    public static Pose deg(double x, double y, double heading) {
        return new Pose(x, y, heading*DEG_TO_RAD);
    }
    public static Pose deg(double heading) {
        return new Pose(0, 0, heading*DEG_TO_RAD);
    }

    /* Redundant
    public static Pose rad(double x, double y, double heading) {
        return new Pose(x, y, heading);
    }
     */

    public static Pose zero() {
        return Pose.zero();
    }

    public static Pose of(double x, double y, double heading) {
        return new Pose(x, y, heading);
    }

    public static Pose of(double heading) {
        return new Pose(0, 0, heading);
    }

    public static Pose of(double x, double y) {
        return new Pose(x, y);
    }

    public static Pose of(com.pedropathing.math.Pose pose) {
        return Pose.fromPedro(pose);
    }

    public static Pose of(Coordinate coordinate) {
        return new Pose(coordinate.getX(), coordinate.getY());
    }

    @FunctionalInterface
    public interface Operation {
        Pose apply(Pose input);
        static Operation identity() {
            return input -> input;
        }
        static Pose convertDegToRad(Pose pose) {
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

    public Pose create(double x, double y, double heading) {
        return operation.apply(of(x, y, heading));
    }

    public Pose create(com.pedropathing.math.Pose pose) {
        return operation.apply(of(pose));
    }

    public Pose create(double x, double y) {
        return operation.apply(of(x, y));
    }
    public Pose create(double heading) {
        return operation.apply(of(heading));
    }

    public Pose create(Coordinate coordinate) {
        return operation.apply(of(coordinate));
    }

}
