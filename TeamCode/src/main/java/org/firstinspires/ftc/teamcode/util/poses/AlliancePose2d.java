package org.firstinspires.ftc.teamcode.util.poses;

import org.firstinspires.ftc.teamcode.util.AllianceColor;

public class AlliancePose2d extends Pose2d {
    Pose2d red;
    Pose2d blue;
    AllianceColor color;
    public AlliancePose2d(Pose2d red, Pose2d blue, AllianceColor defaultColor) {
        super(defaultColor.isBlue()? blue : red);
        this.red = red;
        this.blue = blue;
        this.color = defaultColor;
    }

    public void setAllianceColor(AllianceColor color) {
        if (color.isUnknown()) return;
        this.color = color;
        setAll(color==AllianceColor.RED?red:blue);
    }
}
