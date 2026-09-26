package org.firstinspires.ftc.teamcode.library.game;


import com.pedropathing.geometry.Pose;

public class HiveState {
    public static final HiveState TIPPED = new HiveState(new TagCluster(), new TagCluster());
    public static final HiveState UNTIPPED = new HiveState(new TagCluster(), new TagCluster());
    public Pose goalPose;

    HiveState(TagCluster tagPositionsRed, TagCluster tagPositionsBlue) {

    }

}
