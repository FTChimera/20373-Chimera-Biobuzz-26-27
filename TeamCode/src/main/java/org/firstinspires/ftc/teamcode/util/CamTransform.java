package org.firstinspires.ftc.teamcode.util;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.util.poses.PoseFactory;
import org.firstinspires.ftc.teamcode.util.poses.Pose2d;

public class CamTransform {
    double defaultPitch;
    public CamTransform(double defaultPitch) {
        this.defaultPitch = defaultPitch;
    }
    public double solveForHiveAngle(Pose3D botPoseMt1, double servoPitch) {
        return 0.0; // TODO: wait shouldn't this be snapscript
    }
    public Pose2d solvePose() {return PoseFactory.zero();}
}
