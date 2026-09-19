package org.firstinspires.ftc.teamcode.library;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.library.game.HiveState;

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
    }

    public ShotSolver() {
    }

    // returns angle to face, and velocity inches per second
    public Tuple<Double, Double> calculateShot(ShotContext context) {
        return new Tuple<>(0.0,0.0);
    }

}
