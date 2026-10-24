package org.firstinspires.ftc.teamcode.mechanisms;

import static org.firstinspires.ftc.teamcode.util.math.MathUtils.DEG_TO_RAD;

import com.pedropathing.utils.Angle;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.pedro.commands.TimedScheduler;
import org.firstinspires.ftc.teamcode.util.AllianceColor;
import org.firstinspires.ftc.teamcode.util.CamTransform;
import org.firstinspires.ftc.teamcode.util.game.BiobuzzAprilTags;
import org.firstinspires.ftc.teamcode.util.game.Field;
import org.firstinspires.ftc.teamcode.util.poses.PoseFactory;
import org.firstinspires.ftc.teamcode.util.poses.Pose2d;

public class Vision {
    public Limelight3A limelight3A;
    public WebcamName arducam;
    public final CamTransform camTransform;
    public final BiobuzzAprilTags tags;
    AllianceColor allianceColor;
    public Vision(HardwareMap hardwareMap, AllianceColor allianceColor) {
        limelight3A = hardwareMap.get(Limelight3A.class, "limelight");
        camTransform = new CamTransform(0);
        tags = new BiobuzzAprilTags(allianceColor);
        this.allianceColor = allianceColor;
    }

    // USUAL

    public void start() {limelight3A.start(); limelight3A.pipelineSwitch(0);}

    public LLResult getLatestResult() {
        return limelight3A.getLatestResult();
    }

    public void update() {

    }

    // HIVE/CELL TIPS


    // RELOCALIZATION

    public Pose2d getBotposeMt1() {
        return toPedroPose(limelight3A.getLatestResult().getBotpose());
    }
    public Pose3D getBotposeRaw() { return getLatestResult().getBotpose(); }

    private Pose2d toPedroPose(Pose3D pose3D) {
        return PoseFactory.deg(
                pose3D.getPosition().unit.toInches(pose3D.getPosition().x) - 72,
                pose3D.getPosition().unit.toInches(pose3D.getPosition().y) - 72,
                pose3D.getOrientation().getYaw(AngleUnit.DEGREES) - 90 // TODO confirm
        );
    }

    // AUTOMATED INTAKE DETECTION AND FOLLOWING


    // SHOOTER

    public double getDistance() {
        LLResult result = getLatestResult();
        if (result==null || !result.isValid()) return -1;
        double y = getBestCenterTag(Field.HiveState.TIPPED).getTargetYDegrees()* DEG_TO_RAD; //todo replace placeholder TIPPED
        double camHeight = 10.5; // inches
        double camAngle = 0; // degrees
        double targetHeight = allianceColor.isRed()? 24 : 24; // inches
        return (targetHeight - camHeight) / Math.tan(Math.toRadians(camAngle + y));
    }
    private int[] getCenterTagIds(Field.HiveState hiveState) {
        return hiveState.ordinal() + allianceColor.ordinal() == 1
                ? tags.alliance.getMidTwoTags()
                : tags.opposite.getMidTwoTags();
    }

    public LLResultTypes.FiducialResult getBestCenterTag(
            Field.HiveState hiveState
    ) {
        LLResult result = getLatestResult();

        if (result == null || !result.isValid()) {
            return null;
        }

        int[] centerTagIds = getCenterTagIds(hiveState);

        LLResultTypes.FiducialResult best = null;
        double bestDistanceSquared = Double.POSITIVE_INFINITY;

        for (LLResultTypes.FiducialResult tag
                : result.getFiducialResults()) {

            int id = tag.getFiducialId();

            if (id != centerTagIds[0] && id != centerTagIds[1]) {
                continue;
            }

            Pose3D pose = tag.getTargetPoseCameraSpace();

            if (pose == null) {
                continue;
            }

            double x = pose.getPosition().x;
            double y = pose.getPosition().y;
            double z = pose.getPosition().z;

            double distanceSquared = x * x + y * y + z * z;

            if (Double.isFinite(distanceSquared)
                    && distanceSquared < bestDistanceSquared) {
                bestDistanceSquared = distanceSquared;
                best = tag;
            }
        }

        return best;
    }

}
