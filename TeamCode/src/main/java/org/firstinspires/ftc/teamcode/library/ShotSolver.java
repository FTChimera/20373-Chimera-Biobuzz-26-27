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
    }



}
