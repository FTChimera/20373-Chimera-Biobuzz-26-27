package org.firstinspires.ftc.teamcode.config;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Utility;

import org.firstinspires.ftc.teamcode.planners.StartPoses;
import org.firstinspires.ftc.teamcode.util.poses.PoseFactory;
import org.firstinspires.ftc.teamcode.util.AllianceColor;
import org.firstinspires.ftc.teamcode.util.MatchMode;
import org.firstinspires.ftc.teamcode.util.poses.Pose2d;

@Config
public class OpModeStorage {

    public static Pose2d pose;
    public static AllianceColor allianceColor;
    public static MatchMode matchMode;
    public static boolean initialized = false;

    static {
        resetToDefault();
    }

    public static void setEndVals(Pose2d newPose) {
        pose = newPose;
        initialized = true;
    }

    public static void endMatch() {
        initialized = false;
        resetToDefault();
    }

    /*
    public static void beforeMatch(AllianceColor color, Pose2d newPose) {
        pose = newPose;
        allianceColor = color;
    }
     */

    public static void resetToDefault() {
        pose = StartPoses.gardenArea; // Chimera's testing start pose in robotics room
        allianceColor = AllianceColor.BLUE;
        matchMode = MatchMode.QUALIFICATIONS;
    }

}
