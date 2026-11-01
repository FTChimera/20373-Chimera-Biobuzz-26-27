package org.firstinspires.ftc.teamcode.util.localization;

import static org.junit.Assert.assertEquals;

import org.firstinspires.ftc.teamcode.util.AllianceColor;
import org.firstinspires.ftc.teamcode.util.game.Field;
import org.firstinspires.ftc.teamcode.util.math.MathUtils;
import org.firstinspires.ftc.teamcode.util.poses.Pose2d;
import org.junit.Test;

public class VisionLocalizationMathTest {
    private static final double EPSILON = 1e-9;

    @Test
    public void appliesOffsetsAndHeadingRotation() {
        Pose2d pose = new Pose2d(10.0, 15.0, 0.0);

        Pose2d mapped = VisionLocalizationMath.toFieldPose(
                pose,
                -72.0,
                -72.0,
                -90.0,
                false,
                false,
                false
        );

        assertEquals(-62.0, mapped.x, EPSILON);
        assertEquals(-57.0, mapped.y, EPSILON);
        assertEquals(-90.0 * MathUtils.DEG_TO_RAD, mapped.heading, EPSILON);
    }

    @Test
    public void supportsAxisMirroring() {
        Pose2d pose = new Pose2d(10.0, -15.0, Math.PI / 4.0);

        Pose2d mapped = VisionLocalizationMath.toFieldPose(
                pose,
                2.0,
                3.0,
                180.0,
                true,
                true,
                true
        );

        assertEquals(-8.0, mapped.x, EPSILON);
        assertEquals(18.0, mapped.y, EPSILON);
        assertEquals(3.0 * Math.PI / 4.0, mapped.heading, EPSILON);
    }

    @Test
    public void mapsClustersToHiveStateByAlliance() {
        assertEquals(Field.HiveState.UNTIPPED, VisionLocalizationMath.clusterToFieldHiveState(true, AllianceColor.RED));
        assertEquals(Field.HiveState.TIPPED, VisionLocalizationMath.clusterToFieldHiveState(false, AllianceColor.RED));
        assertEquals(Field.HiveState.TIPPED, VisionLocalizationMath.clusterToFieldHiveState(true, AllianceColor.BLUE));
        assertEquals(Field.HiveState.UNTIPPED, VisionLocalizationMath.clusterToFieldHiveState(false, AllianceColor.BLUE));
    }

    @Test
    public void normalizesHeadingDifferencesAcrossTheWrapBoundary() {
        double currentHeading = Math.toRadians(179.0);
        double visionHeading = Math.toRadians(-179.0);

        assertEquals(Math.toRadians(-2.0),
                VisionLocalizationMath.normalizeRadians(currentHeading - visionHeading),
                EPSILON);
    }

    @Test(expected = IllegalArgumentException.class)
    public void rejectsUnknownAllianceForClusterMapping() {
        VisionLocalizationMath.clusterToFieldHiveState(true, AllianceColor.UNKNOWN);
    }
}
