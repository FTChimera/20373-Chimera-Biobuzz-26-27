package org.firstinspires.ftc.teamcode.opModes.variants;

import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.library.AllianceColor;
import org.firstinspires.ftc.teamcode.opModes.IvyAuton;

public class Test_IvyAuton extends IvyAuton {
    // if doesnt work make paths abstract class
    static class Paths extends IvyAuton_Paths {
        public static Paths INSTANCE = new Paths();
        Pose STARTING_POSE = new Pose(36, 12, Math.toRadians(180));
        Pose endPose = new Pose(36, 36, Math.toRadians(180));

        @Override
        public PathChain getPath1() {
            return null;
        }

        @Override
        public Pose getStartingPose() {
            return null;
        }
    }
    public Test_IvyAuton() {
        super(AllianceColor.RED, Paths.INSTANCE);
    }

}
