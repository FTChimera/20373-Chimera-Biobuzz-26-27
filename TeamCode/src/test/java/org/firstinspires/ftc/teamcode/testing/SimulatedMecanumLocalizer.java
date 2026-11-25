package org.firstinspires.ftc.teamcode.testing;

import com.pedropathing.localization.Localizer;
import com.pedropathing.localization.MotionState;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import java.util.Objects;

/**
 * Ideal planar odometry from logical wheel encoder speeds, not commanded powers.
 * Units are inches, radians and seconds; wheel order is FL, FR, BL, BR.
 */
public final class SimulatedMecanumLocalizer implements Localizer {
    private final DcMotorEx[] motors;
    private final double wheelFreeSpeed;
    private final double forwardSpeed;
    private final double strafeSpeed;
    private final double angularSpeed;
    private Pose pose = Pose.zero();
    private Velocity velocity = Velocity.zero();

    public SimulatedMecanumLocalizer(DcMotorEx frontLeft, DcMotorEx frontRight,
                                    DcMotorEx backLeft, DcMotorEx backRight,
                                    double wheelFreeSpeed, double forwardSpeed,
                                    double strafeSpeed, double angularSpeed) {
        motors = new DcMotorEx[]{frontLeft, frontRight, backLeft, backRight};
        for (DcMotorEx motor : motors) Objects.requireNonNull(motor, "wheel motor");
        this.wheelFreeSpeed = positive(wheelFreeSpeed);
        this.forwardSpeed = positive(forwardSpeed);
        this.strafeSpeed = positive(strafeSpeed);
        this.angularSpeed = positive(angularSpeed);
    }

    public void advance(double seconds) {
        if (!Double.isFinite(seconds) || seconds <= 0) {
            throw new IllegalArgumentException("Simulation step must be finite and positive");
        }
        double fl = motors[0].getVelocity() / wheelFreeSpeed;
        double fr = motors[1].getVelocity() / wheelFreeSpeed;
        double bl = motors[2].getVelocity() / wheelFreeSpeed;
        double br = motors[3].getVelocity() / wheelFreeSpeed;
        double forward = (fl + fr + bl + br) * forwardSpeed / 4;
        double strafe = (-fl + fr + bl - br) * strafeSpeed / 4;
        double turn = (-fl + fr - bl + br) * angularSpeed / 4;
        double midpointHeading = pose.heading() + turn * seconds / 2;
        double vx = forward * Math.cos(midpointHeading) - strafe * Math.sin(midpointHeading);
        double vy = forward * Math.sin(midpointHeading) + strafe * Math.cos(midpointHeading);
        pose = new Pose(pose.x() + vx * seconds, pose.y() + vy * seconds,
                pose.heading() + turn * seconds);
        velocity = new Velocity(vx, vy, turn);
    }

    @Override public void setPose(Pose pose) {
        this.pose = Objects.requireNonNull(pose, "pose");
        velocity = Velocity.zero();
    }

    @Override public MotionState state() { return MotionState.ofVelocity(pose, velocity); }
    @Override public void update() { }
    @Override public void reset() { setPose(Pose.zero()); }

    private static double positive(double value) {
        if (!Double.isFinite(value) || value <= 0) {
            throw new IllegalArgumentException("Plant speed must be finite and positive");
        }
        return value;
    }
}
