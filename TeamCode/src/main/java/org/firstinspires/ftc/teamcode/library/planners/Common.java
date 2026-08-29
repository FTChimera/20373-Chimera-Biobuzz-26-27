package org.firstinspires.ftc.teamcode.library.planners;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.library.AllianceColor;


public class Common {
    public Pose score1red=new Pose();
    public Pose score1blue=new Pose();

    public Pose getScore1Loc(AllianceColor col) {
        if (col.isRed()) return score1red;
        return score1blue;
    }
}
