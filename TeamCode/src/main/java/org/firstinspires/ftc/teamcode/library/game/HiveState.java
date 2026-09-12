package org.firstinspires.ftc.teamcode.library.game;


public class HiveState {
    public static final HiveState TIPPED = new HiveState(new TagCluster(), new TagCluster());
    public static final HiveState UNTIPPED = new HiveState(new TagCluster(), new TagCluster());

    HiveState(TagCluster tagPositionsRed, TagCluster tagPositionsBlue) {

    }

}
