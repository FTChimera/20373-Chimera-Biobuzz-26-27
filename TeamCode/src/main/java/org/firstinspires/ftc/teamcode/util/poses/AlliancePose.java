package org.firstinspires.ftc.teamcode.util.poses;

import com.rmsftc.utils.AllianceColor;
import com.rmsftc.utils.math.Pose;

public class AlliancePose extends Pose {
    Pose red;
    Pose blue;
    AllianceColor color;

    private static Pose initial(Pose red, Pose blue, AllianceColor defaultColor) {
        return defaultColor.isBlue() ? blue : red;
    }

    public AlliancePose(Pose red, Pose blue, AllianceColor defaultColor) {
        super(initial(red, blue, defaultColor).x(),
                initial(red, blue, defaultColor).y(),
                initial(red, blue, defaultColor).heading());
        this.red = red;
        this.blue = blue;
        this.color = defaultColor;
    }

    public void setAllianceColor(AllianceColor color) {
        if (color.isUnknown()) return;
        this.color = color;
        Pose target = color==AllianceColor.RED?red:blue;
        set(target.x(), target.y(), target.heading());
    }
}
