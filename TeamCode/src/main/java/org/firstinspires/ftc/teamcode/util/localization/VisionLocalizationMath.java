package org.firstinspires.ftc.teamcode.util.localization;

import org.firstinspires.ftc.teamcode.util.AllianceColor;
import org.firstinspires.ftc.teamcode.util.game.Field;
import org.firstinspires.ftc.teamcode.util.poses.Pose2d;
import org.firstinspires.ftc.teamcode.util.poses.PoseFactory;

public final class VisionLocalizationMath {
    private VisionLocalizationMath() {
    }

    public static Pose2d toFieldPose(
            Pose2d limelightPoseInches,
            double xOffsetIn,
            double yOffsetIn,
            double headingOffsetDeg,
            boolean invertX,
            boolean invertY,
            boolean invertHeading
    ) {
        if (limelightPoseInches == null) {
            throw new IllegalArgumentException("Limelight pose is required.");
        }
        requireFinite(xOffsetIn, "xOffsetIn");
        requireFinite(yOffsetIn, "yOffsetIn");
        requireFinite(headingOffsetDeg, "headingOffsetDeg");

        double x = limelightPoseInches.x;
        double y = limelightPoseInches.y;
        double heading = limelightPoseInches.heading;

        if (invertX) {
            x = -x;
        }
        if (invertY) {
            y = -y;
        }
        if (invertHeading) {
            heading = -heading;
        }

        heading += Math.toRadians(headingOffsetDeg);
        return PoseFactory.of(x + xOffsetIn, y + yOffsetIn, normalizeRadians(heading));
    }

    public static Field.HiveState clusterToFieldHiveState(boolean allianceClusterDetected, AllianceColor allianceColor) {
        if (allianceColor == null || allianceColor.isUnknown()) {
            throw new IllegalArgumentException("Alliance color must be RED or BLUE.");
        }
        if (allianceClusterDetected) {
            return allianceColor.isRed() ? Field.HiveState.UNTIPPED : Field.HiveState.TIPPED;
        }
        return allianceColor.isRed() ? Field.HiveState.TIPPED : Field.HiveState.UNTIPPED;
    }

    public static double normalizeRadians(double angle) {
        if (!Double.isFinite(angle)) {
            throw new IllegalArgumentException("Heading angle must be finite.");
        }
        double wrapped = (angle + Math.PI) % (2.0 * Math.PI);
        if (wrapped < 0.0) {
            wrapped += 2.0 * Math.PI;
        }
        return wrapped - Math.PI;
    }

    private static void requireFinite(double value, String name) {
        if (!Double.isFinite(value)) {
            throw new IllegalArgumentException(name + " must be finite.");
        }
    }
}
