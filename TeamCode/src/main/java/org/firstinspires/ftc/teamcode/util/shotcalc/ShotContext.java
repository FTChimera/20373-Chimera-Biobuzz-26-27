package org.firstinspires.ftc.teamcode.util.shotcalc;



import com.pedropathing.math.Velocity;

import org.firstinspires.ftc.teamcode.util.AllianceColor;
import org.firstinspires.ftc.teamcode.util.game.BallCol;
import org.firstinspires.ftc.teamcode.util.game.Field;
import org.firstinspires.ftc.teamcode.util.poses.Pose2d;

public class ShotContext {
    public Pose2d robotPose;
    public Velocity velocity;
    public double dt;

    public Field.HiveState hiveState;
    public AllianceColor allianceColor;
    public BallCol transferBallCol;
    public boolean sotm;

    public double distanceToTarget() {
        return robotPose.plus(velocity.toVector2D().toPose()).distance(hiveState.goalPose(allianceColor));
    }

    public double angleToTarget() {
        return 0;
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
