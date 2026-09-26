package org.firstinspires.ftc.teamcode.library;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.library.game.HiveState;
import org.firstinspires.ftc.teamcode.library.math.MathUtils;

public class ShotSolver {
    public static class ShotContext {
        public Pose compensatedPose;
        public HiveState hiveState;

        public ShotContext(Pose pose, HiveState state) {
            compensatedPose = pose;
            hiveState = state;
        }

        public static class Builder {
            public ShotContext c;
            public static Builder create() {return new Builder();}
            public Builder() {c=new ShotContext(null,null);}
            public Builder pose(Pose pose) {c.compensatedPose=pose.copy();return this;}
            public Builder state(HiveState state) {c.hiveState=state;return this;}
        }

        public boolean isSameAs(ShotContext other) {
            return other.compensatedPose.roughlyEquals(this.compensatedPose, MathUtils.EPSILON) && this.hiveState == other.hiveState;
        }
    }

    public ShotSolver() {
    }
    // This saves loop times
    public Tuple<Double, Double> cached;
    public ShotContext cachedContext;
    // returns angle to face, and velocity inches per second
    public Tuple<Double, Double> calculateShot(ShotContext context) {
        // This saves loop times
        if (cached != null && cachedContext != null) {
            if (context.isSameAs(cachedContext)) {
                return cached;
            }
        }
        cachedContext = context;
        Pose toGoal = context.compensatedPose.minus(context.hiveState.goalPose);
        double distanceToGoal = hypot(toGoal);
        double angleToGoal = Math.atan2(toGoal.getY(), toGoal.getX());
        cached = new Tuple<>(angleToGoal-context.compensatedPose.getHeading(),0.0);
        return cached;
    }
    // This also saves loop times; hypot is slow for no reason
    private double hypot(Pose posDiff) {
        return Math.sqrt(Math.pow(posDiff.getX(),2) + Math.pow(posDiff.getY(),2));
    }

}
