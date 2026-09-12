package org.firstinspires.ftc.teamcode.library.planners;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.library.AllianceColor;
import org.firstinspires.ftc.teamcode.mechanisms.Robot;
import org.firstinspires.ftc.teamcode.pedroPathing.AlliancePoses;


public class Common {
    public static final AlliancePoses start = new AlliancePoses(new Pose(), new Pose());
    public static final AlliancePoses score = new AlliancePoses(new Pose(), new Pose());
    public static final AlliancePoses intake;
    public static final AlliancePoses intakeTest = new AlliancePoses(new Pose(), new Pose());

    public static void init(AllianceColor color) {
        start.setAlliance(color);
        score.setAlliance(color);
    }
}
