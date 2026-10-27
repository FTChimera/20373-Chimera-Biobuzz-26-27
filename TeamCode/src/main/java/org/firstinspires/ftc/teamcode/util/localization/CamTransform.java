package org.firstinspires.ftc.teamcode.util.localization;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.util.AllianceColor;
import org.firstinspires.ftc.teamcode.util.math.MathUtils;
import org.firstinspires.ftc.teamcode.util.poses.Pose2d;

import java.util.List;

@Config
public final class CamTransform {
    public static final int MIDDLE = 0;
    public static final int AUDIENCE_UP = 1;
    public static final int AUDIENCE_DOWN = 2;
    public static final int UNKNOWN = 3;
    public static boolean isScoreable(int state) {return state > 0 && state < 3;}
    public static boolean isValid(int state) {return state >= 0 && state < 3;}
    public static boolean isTippingCurrently(int state) {return state==0;}
    private final Limelight3A limelight;

    private static final int PIPELINE_RED = 0;
    private static final int PIPELINE_BLUE = 1;

    private LLResult latestResult;
    private Pose3D botpose;
    private boolean valid;

    private double[] stdDevs;
    private int tagCount;
    private double avgDist;
    private double avgArea;
    private double span;
    private double timestamp;
    private double captureLatency;
    private double timestampNano;

    private double lastFrameTimestamp = Double.NaN;

    private double maxFiducialSkew;

    private double rawXIn;
    private double rawYIn;
    private double rawZIn;
    private double rawYaw;
    private double rawPitch;

    private double hiveAngle = Double.NaN;
    private int hiveState = UNKNOWN;
    private boolean hiveEstimated = false;
    private double correctedXIn;
    private double correctedYIn;
    private double correctedYaw;
    private double pitchCheckErrDeg = Double.NaN;

    public CamTransform(Limelight3A limelight, AllianceColor teamColor) {
        this.limelight = limelight;
        this.valid = false;
        this.stdDevs = new double[6];
        limelight.pipelineSwitch(teamColor == AllianceColor.BLUE ? PIPELINE_BLUE : PIPELINE_RED);
    }


    public void update() {
        latestResult = limelight.getLatestResult();

        if (latestResult == null || !latestResult.isValid()) {
            valid = false;
            resetHiveObservation();
            return;
        }

        botpose = latestResult.getBotpose();
        if (botpose == null) {
            valid = false;
            resetHiveObservation();
            return;
        }

        double[] mt1StdDevs = latestResult.getStddevMt1();
        if (mt1StdDevs != null && mt1StdDevs.length >= stdDevs.length) {
            stdDevs = mt1StdDevs;
        }
        tagCount = latestResult.getBotposeTagCount();
        avgDist = latestResult.getBotposeAvgDist();
        avgArea = latestResult.getBotposeAvgArea();
        span = latestResult.getBotposeSpan();
        double frameTimestamp = latestResult.getTimestamp();
        boolean isNewFrame = frameTimestamp != lastFrameTimestamp;
        lastFrameTimestamp = frameTimestamp;
        timestamp = frameTimestamp;

        captureLatency = latestResult.getCaptureLatency();
        if (isNewFrame) {
            timestampNano = System.nanoTime() / 1e9 - captureLatency / 1000.0;
        }

        maxFiducialSkew = 0.0;
        List<LLResultTypes.FiducialResult> fiducials = latestResult.getFiducialResults();
        if (fiducials != null) {
            for (LLResultTypes.FiducialResult fr : fiducials) {
                double skew = fr.getSkew();
                if (skew > maxFiducialSkew) {
                    maxFiducialSkew = skew;
                }
            }
        }

        rawXIn = MathUtils.units.toInches(botpose.getPosition().x, MathUtils.DistanceUnit.METERS);
        rawYIn = MathUtils.units.toInches(botpose.getPosition().y, MathUtils.DistanceUnit.METERS);
        rawZIn = MathUtils.units.toInches(botpose.getPosition().z, MathUtils.DistanceUnit.METERS);
        rawYaw = botpose.getOrientation().getYaw(AngleUnit.RADIANS);
        rawPitch = botpose.getOrientation().getPitch(AngleUnit.RADIANS);

        valid = true;

        solveHiveObservation();
    }


    private void solveHiveObservation() {
        resetHiveObservation();

        final double h = HypParams.hivePivotHeightIn;
        if (!(h > 0)) {
            return;
        }

        final double a = rawXIn;
        final double b = rawZIn - h;
        final double c = -h;
        final double radius = Math.hypot(a, b);

        if (radius < 1e-9 || Math.abs(c) > radius) {
            return;
        }

        final double phi = solvePhi(a, b, c, Math.toRadians(HypParams.hiveCellUpAngleDeg));
        if (Double.isNaN(phi)) {
            return;
        }

        final double sinPredicted = Math.sin(phi) * Math.cos(rawYaw);
        final double predictedPitch = Math.asin(Math.max(-1.0, Math.min(1.0, sinPredicted)));
        pitchCheckErrDeg = Math.toDegrees(Math.abs(normalize(rawPitch - predictedPitch)));
        if (pitchCheckErrDeg > HypParams.hivePitchCheckTolDeg) {
            return;
        }

        final double cosPhi = Math.cos(phi);
        final double sinPhi = Math.sin(phi);
        correctedXIn = a * cosPhi - rawZIn * sinPhi + h * sinPhi;
        correctedYIn = rawYIn;

        final double r00 = Math.cos(rawYaw) * Math.cos(rawPitch);
        final double r10 = Math.sin(rawYaw) * Math.cos(rawPitch);
        final double r20 = -Math.sin(rawPitch);
        correctedYaw = Math.atan2(r10, cosPhi * r00 - sinPhi * r20);

        hiveAngle = -phi;                       // θ = -φ
        hiveState = classifyHive(hiveAngle);
        hiveEstimated = true;
    }

    private static double solvePhi(double a, double b, double c, double maxTilt) {
        final double radius = Math.hypot(a, b);
        final double base = Math.atan2(b, a);
        final double alpha = Math.asin(c / radius);

        final double phi1 = normalize(alpha - base);
        final double phi2 = normalize(Math.PI - alpha - base);

        final boolean in1 = Math.abs(phi1) <= maxTilt;
        final boolean in2 = Math.abs(phi2) <= maxTilt;
        if (in1 && in2) {
            return Math.abs(phi1) <= Math.abs(phi2) ? phi1 : phi2;
        }
        if (in1) {
            return phi1;
        }
        if (in2) {
            return phi2;
        }
        return Double.NaN;
    }

    private static int classifyHive(double theta) {
        if (Math.abs(theta) <= Math.toRadians(HypParams.hiveCellDownAngleDeg)) {
            return MIDDLE;
        }
        final boolean up = HypParams.hivePositiveAngleIsAudienceUp == (theta > 0);
        return up ? AUDIENCE_UP : AUDIENCE_DOWN;
    }

    private static double normalize(double angle) {
        double result = (angle + Math.PI) % (2 * Math.PI);
        if (result < 0) {
            result += 2 * Math.PI;
        }
        return result - Math.PI;
    }

    private void resetHiveObservation() {
        hiveEstimated = false;
        hiveAngle = Double.NaN;
        hiveState = UNKNOWN;
        pitchCheckErrDeg = Double.NaN;
    }



    public Pose2d getPose() {
        if (!valid || botpose == null) {
            return new Pose2d(0, 0, 0);
        }
        if (hiveEstimated) {
            return new Pose2d(correctedXIn, correctedYIn, correctedYaw);
        }
        return getRawPose();
    }

    public void setPose(Pose2d pose) {
        // no-op
    }

    public Pose2d getRawPose() {
        if (!valid || botpose == null) {
            return new Pose2d(0, 0, 0);
        }
        return new Pose2d(rawXIn, rawYIn, rawYaw);
    }

    public double getRawZIn() {
        return rawZIn;
    }

    public double getRawPitch() {
        return rawPitch;
    }

    public String getTagIds() {
        if (latestResult == null) {
            return "";
        }
        List<LLResultTypes.FiducialResult> fiducials = latestResult.getFiducialResults();
        if (fiducials == null) {
            return "";
        }
        StringBuilder ids = new StringBuilder();
        for (LLResultTypes.FiducialResult fr : fiducials) {
            if (ids.length() > 0) {
                ids.append(',');
            }
            ids.append(fr.getFiducialId());
        }
        return ids.toString();
    }

    public double[] getPoseArray() {
        Pose2d pose = getPose();
        return new double[]{pose.x, pose.y, pose.heading};
    }

    public Pose3D getBotpose() {
        return botpose;
    }

    public double[] getStdDevs() {
        return stdDevs.clone();
    }



    public double getHiveAngle() {
        return hiveAngle;
    }


    public int getHiveState() {
        return hiveState;
    }


    public boolean isHiveEstimated() {
        return hiveEstimated;
    }

    public double getHivePitchCheckErrDeg() {
        return pitchCheckErrDeg;
    }


    public double getAmbiguity() {
        if (!valid || stdDevs == null || stdDevs.length < 2) {
            return Double.MAX_VALUE;
        }
        return Math.sqrt(stdDevs[0] * stdDevs[0] + stdDevs[1] * stdDevs[1]);
    }

    public double getAngularAmbiguity() {
        if (!valid || stdDevs == null || stdDevs.length < 6) {
            return Double.MAX_VALUE;
        }
        return Math.toRadians(stdDevs[5]);
    }


    public int getTagCount() {
        return tagCount;
    }

    public double getAvgDist() {
        return avgDist;
    }

    public double getAvgArea() {
        return avgArea;
    }

    public double getSpan() {
        return span;
    }

    public double getMaxFiducialSkew() {
        return maxFiducialSkew;
    }

    public double getTimestamp() {
        return timestamp;
    }

    public double getCaptureLatency() {
        return captureLatency;
    }


    public double getTimestampNanoBase() {
        return timestampNano;
    }

    public boolean isValid() {
        return valid;
    }



    public boolean isReliable(double ambiguityThreshold) {
        return valid && tagCount >= 2 && getAmbiguity() < ambiguityThreshold;
    }
    public static class HypParams {
        public static double hivePivotHeightIn = 43.95;
        public static double hiveCellDownAngleDeg = 23.0;
        public static double hiveCellUpAngleDeg = 35.0;
        public static double hivePitchCheckTolDeg = 10.0;
        public static boolean hivePositiveAngleIsAudienceUp = Boolean.FALSE;
    }
}