package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.library.AllianceColor;

import java.util.Optional;

/** Poses specific to each alliance (mirrored) */
public class AlliancePoses {
    Pose redAlliancePose;
    Pose blueAlliancePose;
    AllianceColor color=AllianceColor.RED;

    public AlliancePoses(Pose redPose, Pose bluePose) {
        redAlliancePose = redPose; blueAlliancePose = bluePose;
    }

    public Pose Red() {
        return redAlliancePose.copy();
    }
    public Pose Blue() {
        return blueAlliancePose.copy();
    }
    public Pose pose() {return color.isRed()?Red():Blue();
    }

    public void setAlliance(AllianceColor color) {
        this.color = color;
    }


}
