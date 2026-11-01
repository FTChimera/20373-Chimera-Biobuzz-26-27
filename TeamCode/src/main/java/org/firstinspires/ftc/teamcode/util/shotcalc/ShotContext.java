package org.firstinspires.ftc.teamcode.util.shotcalc;



import com.pedropathing.math.Velocity;

import org.firstinspires.ftc.teamcode.util.AllianceColor;
import org.firstinspires.ftc.teamcode.util.game.BallCol;
import org.firstinspires.ftc.teamcode.util.game.Field;
import org.firstinspires.ftc.teamcode.util.poses.Pose2d;

import java.util.Objects;

public class ShotContext {
    public Pose2d robotPose;
    public Velocity velocity;
    public double dt;

    public Field.HiveState hiveState;
    public AllianceColor allianceColor;
    public BallCol transferBallCol;
    public boolean sotm;

    public double distanceToTarget() {
        validate();
        return robotPose.distance(targetPose());
    }

    public double angleToTarget() {
        validate();
        return Math.atan2(targetPose().y() - robotPose.y(), targetPose().x() - robotPose.x());
    }

    public Pose2d targetPose() {
        validateBaseFields();
        return new Pose2d(hiveState.goalPose(allianceColor));
    }

    /**
     * Predicts the pose at ball release. The horizon is the measured control-loop
     * latency, not projectile flight time.
     */
    public Pose2d poseAtRelease() {
        validate();
        return sotm ? robotPose.exp(velocity, dt) : robotPose.copy();
    }

    public void validate() {
        validateBaseFields();
        if (!Double.isFinite(dt) || dt < 0.0) {
            throw new IllegalArgumentException("Shot calculation latency must be finite and nonnegative.");
        }
        if (sotm) {
            Objects.requireNonNull(velocity, "Velocity is required when shoot-on-move is enabled.");
            if (!Double.isFinite(velocity.vx) || !Double.isFinite(velocity.vy)
                    || !Double.isFinite(velocity.omega)) {
                throw new IllegalArgumentException("Shoot-on-move velocity must be finite.");
            }
        }
    }

    private void validateBaseFields() {
        Objects.requireNonNull(robotPose, "Robot pose is required for shot calculation.");
        Objects.requireNonNull(hiveState, "Hive state is required for shot calculation.");
        Objects.requireNonNull(allianceColor, "Alliance color is required for shot calculation.");
        Objects.requireNonNull(transferBallCol, "Transfer ball color is required for shot calculation.");
    }

    public ShotContext copy() {
        ShotContext copy = new ShotContext();
        copy.robotPose = robotPose;
        copy.velocity = velocity;
        copy.hiveState = hiveState;
        copy.allianceColor = allianceColor;
        copy.transferBallCol = transferBallCol;
        copy.sotm = sotm;
        copy.dt = dt;
        return copy;
    }
}
