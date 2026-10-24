package org.firstinspires.ftc.teamcode.util.poses;

import androidx.annotation.NonNull;

import com.pedropathing.math.Matrix;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Twist;
import com.pedropathing.math.Vector2D;
import com.pedropathing.math.Velocity;

import org.firstinspires.ftc.teamcode.util.math.MathUtils;
import org.locationtech.jts.geom.Coordinate;
import org.locationtech.jts.geom.Geometry;

import java.util.Locale;

// Heres the plan:
// we extend Pose, then we override everything so that it becomes like our custom class
public class Pose2d extends Pose {
    public double x, y, heading;
    public static final Pose2d ZERO = new Pose2d();

    public Pose2d() {super(0,0,0);}
    public Pose2d(double x, double y, double heading) {
        this();
        this.x = x;
        this.y = y;
        this.heading = heading;
    }
    public Pose2d(double x, double y) {
        this(x, y, 0);
    }
    public Pose2d(Pose other) {
        this(other.x(), other.y(), other.heading());
    }

    @Override
    public double x() {
        return this.x;
    }

    @Override
    public double y() {
        return this.y;
    }

    @Override
    public double heading() {
        return this.heading;
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public double getHeading() {
        return heading;
    }

    public void setX(double x) {
        this.x = x;
    }

    public void setY(double y) {
        this.y = y;
    }

    public void setHeading(double heading) {
        this.heading = heading;
    }

    public void setAll(Pose2d other) {
        setX(other.getX());
        setY(other.getY());
        setHeading(other.getHeading());
    }

    @Override
    public Pose2d withX(double x) {
        return new Pose2d(x, y(), heading());
    }

    @Override
    public Pose2d withY(double y) {
        return new Pose2d(x(), y, heading());
    }
    @Override
    public Pose2d withHeading(double heading) {
        return new Pose2d(x(), y(), heading);
    }

    @Override
    public Vector2D toVector2D() {
        return Vector2D.cartesian(x(), y());
    }

    public Coordinate toCoordinate() {return new Coordinate(x(), y());}

    @Override
    public Matrix toMatrix() {
        return Matrix.createTransformation(x(), y(), heading());
    }

    @Override
    public Pose2d exp(Velocity velocity, double time) {
        return new Pose2d(this.x + velocity.vx * time, this.y + velocity.vy * time, this.heading + velocity.omega * time);
    }

    @Override
    public Pose2d exp(Velocity velocity) {
        return this.exp(velocity, (double)1.0F);
    }

    @Override
    public Pose2d exp(Twist twist, double time) {
        if (Math.abs(twist.omega) < 1.0E-9) {
            return this.exp(twist.toVelocity(this.heading), time);
        } else {
            double theta = twist.omega * time;
            double sin = Math.sin(theta);
            double cos = Math.cos(theta);
            Vector2D localDeltas = Vector2D.cartesian((sin * twist.vx - ((double)1.0F - cos) * twist.vy) / twist.omega, (((double)1.0F - cos) * twist.vx + sin * twist.vy) / twist.omega);
            Vector2D globalDeltas = localDeltas.rotate(this.heading);
            return new Pose2d(this.x + globalDeltas.x(), this.y + globalDeltas.y(), this.heading + theta);
        }
    }

    @Override
    public Pose2d exp(Twist twist) {
        return this.exp(twist, (double)1.0F);
    }

    @Override
    public Pose2d compose(Pose other) {
        Vector2D translationDeltas = other.toVector2D().rotate(this.heading);
        return new Pose2d(this.x + translationDeltas.x(), this.y + translationDeltas.y(), this.heading + other.heading());
    }

    @Override
    public Pose2d plus(Pose other) {
        return new Pose2d(this.x + other.x(), this.y + other.y(), this.heading + other.heading());
    }

    public Pose2d minus(Pose other) {
        return new Pose2d(this.x - other.x(), this.y - other.y(), this.heading - other.heading());
    }

    @Override
    public Pose2d times(double scalar) {
        return new Pose2d(this.x * scalar, this.y * scalar, this.heading * scalar);
    }

    public Pose2d div(double scalar) {
        return new Pose2d(this.x / scalar, this.y / scalar, this.heading / scalar);
    }

    public double distance(Pose other) {
        return Math.hypot(this.x - other.x(), this.y - other.y());
    }

    public Pose2d invert() {
        Vector2D invTrans = Vector2D.cartesian(-this.x, -this.y).rotate(-this.heading);
        return new Pose2d(invTrans.x(), invTrans.y(), -this.heading);
    }

    public Twist log() {
        double eps = 1.0E-6;
        if (Math.abs(this.heading) < eps) {
            return new Twist(this.x, this.y, this.heading);
        } else {
            double A = Math.sin(this.heading) / this.heading;
            double B = ((double)1.0F - Math.cos(this.heading)) / this.heading;
            double denom = A * A + B * B;
            double vx = (A * this.x + B * this.y) / denom;
            double vy = (-B * this.x + A * this.y) / denom;
            return new Twist(vx, vy, this.heading);
        }
    }

    public Pose2d copy() {
        return new Pose2d(this);
    }

    @NonNull
    public String toString() {
        return String.format(Locale.US, "(%.2f, %.2f, %.2f)", this.x, this.y, this.heading/ MathUtils.DEG_TO_RAD);
    }
}
