package org.firstinspires.ftc.teamcode.library.planners;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.library.AllianceColor;
import org.firstinspires.ftc.teamcode.pedroPathing.AlliancePoses;


public class Common {
    public static class Timings {
        public static final double flowerIntakeTime = 3;
    }
    public static class Values {
        // T-Values
        public static final double TintakeGardenTangentialStart = 0.2;
        public static final double TintakeGardenTangentialEnd = 0.8;

        // Drive-Powers
    }

    public static class StartPoses {
        public static AlliancePoses garden = new AlliancePoses(new Pose(), new Pose());
        public static AlliancePoses loading = new AlliancePoses(new Pose(), new Pose());
        public static AlliancePoses visionAnchorGarden = new AlliancePoses(new Pose(), new Pose());
        public static AlliancePoses visionAnchorLoading = new AlliancePoses(new Pose(), new Pose());
        static void init(AllianceColor color) {
            garden.setAlliance(color);
            loading.setAlliance(color);
        }
    }
    public static class IntakePoses {
        public static AlliancePoses gardenEnd = new AlliancePoses(new Pose(), new Pose());
        public static AlliancePoses gardenControl1 = new AlliancePoses(new Pose(), new Pose());
        public static AlliancePoses gardenControl2 = new AlliancePoses(new Pose(), new Pose());


        static void init(AllianceColor color) {
            gardenEnd.setAlliance(color);
            gardenControl1.setAlliance(color);
            gardenControl2.setAlliance(color);
        }
    }
    public static class ShootPoses {
        public static AlliancePoses garden = new AlliancePoses(new Pose(), new Pose());
        public static AlliancePoses loading = new AlliancePoses(new Pose(), new Pose());

        static void init(AllianceColor color) {
            garden.setAlliance(color);
            loading.setAlliance(color);
        }
    }

    public static final AlliancePoses intake = new AlliancePoses(new Pose(), new Pose());
    public static final AlliancePoses park = new AlliancePoses(new Pose(), new Pose());
    public static final AlliancePoses intakeTest = new AlliancePoses(new Pose(), new Pose());

    public static void init(AllianceColor color) {
        StartPoses.init(color);
        IntakePoses.init(color);
        ShootPoses.init(color);
    }
}
