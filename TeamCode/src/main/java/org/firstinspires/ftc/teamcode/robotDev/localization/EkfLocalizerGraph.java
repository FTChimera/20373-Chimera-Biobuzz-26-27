package org.firstinspires.ftc.teamcode.robotDev.localization;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Pose;
import com.pedropathing.utils.Angle;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Utility;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.pedro.Follower;
import org.firstinspires.ftc.teamcode.planners.UtilityPoses;
import org.firstinspires.ftc.teamcode.util.hardware.gamepad.DriveVector;
import org.firstinspires.ftc.teamcode.util.math.MathUtils;

import java.util.Map;

/**
 * Streams the EKF localizer to FTCDashboard so it can be graphed
 * (Dashboard -> graph: add each series by name).
 *
 * <p>This uses {@link Constants#createEKFFollower}, which wraps the
 * Pinpoint odometry in {@code EKFPinpoint} — a constant-velocity
 * Extended Kalman Filter (state [x, y, heading, vx, vy, omega]) that
 * smooths the raw Pinpoint pose. Drive the robot with the gamepad
 * sticks (field-centric, like PoseCollector) and watch the EKF pose
 * track the raw pose with less jitter, while the covariance trace
 * falls as the filter gains confidence.
 *
 * <p>Graph in FTCDashboard by adding these series (all numeric):
 * ekfX / rawX, ekfY / rawY, ekfHeadingDeg / rawHeadingDeg (overlay each
 * EKF series against its raw counterpart to see the smoothing),
 * innovX / innovY / innovHeadingDeg (raw minus EKF: the measurement
 * innovation, i.e. how far each new Pinpoint reading pulls the EKF),
 * covTrace (sum of the covariance diagonal: falls as the filter
 * converges), covX / covY / covHeading (per-axis variance).
 *
 * <p>Note: the robot's production follower ({@code Robot}) uses a plain
 * Pinpoint localizer, not the EKF; this OpMode exists to exercise and
 * evidence the EKF itself. The EKF pose is the localizer pose, so
 * {@code follower.localizer.pose()} is the filtered pose and the raw
 * Pinpoint pose comes from {@code localizer.debug()} ("rawPose"), with
 * the covariance from the same debug map ("covariance").
 */
@Config
@Utility(name = "EKF Localizer Graph")
public class EkfLocalizerGraph extends LinearOpMode {

    /** Stick scale for the field-centric drive (same as PoseCollector). */
    public static double DRIVE_SCALE = 0.4;

    @Override
    public void runOpMode() throws InterruptedException {
        Follower follower = Constants.createEKFFollower(hardwareMap);
        follower.localizer.setPose(UtilityPoses.cornerGardenArea);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addLine("EKF localizer graph. Drive with gamepad 1 sticks (field-centric).");
        telemetry.addLine("Graph: ekfX/rawX, ekfY/rawY, ekfHeadingDeg/rawHeadingDeg, innovX/innovY/innovHeadingDeg, covTrace, covX/covY/covHeading.");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) {
            return;
        }

        try {
            while (opModeIsActive()) {
                follower.manual(ManualDrive.fieldCentric(
                        new DriveVector(
                                -gamepad1.left_stick_y * DRIVE_SCALE,
                                -gamepad1.left_stick_x * DRIVE_SCALE,
                                -gamepad1.right_stick_x * DRIVE_SCALE),
                        follower.localizer.pose().heading()));
                follower.update();

                Pose ekf = follower.localizer.pose();
                Map<String, Object> debug = follower.localizer.debug();
                Pose raw = ekf;
                Matrix covariance = null;
                if (debug != null) {
                    Object rawObj = debug.get("rawPose");
                    if (rawObj instanceof Pose) {
                        raw = (Pose) rawObj;
                    }
                    Object covObj = debug.get("covariance");
                    if (covObj instanceof Matrix) {
                        covariance = (Matrix) covObj;
                    }
                }

                double covTrace = 0.0;
                double covX = 0.0;
                double covY = 0.0;
                double covHeading = 0.0;
                if (covariance != null && covariance.numRows() >= 3 && covariance.numCols() >= 3) {
                    covX = covariance.get(0, 0);
                    covY = covariance.get(1, 1);
                    covHeading = covariance.get(2, 2);
                    covTrace = covX + covY + covHeading;
                    // Include the velocity variances in the trace as well.
                    for (int i = 3; i < Math.min(covariance.numRows(), covariance.numCols()); i++) {
                        covTrace += covariance.get(i, i);
                    }
                }

                telemetry.addData("ekfX", ekf.x());
                telemetry.addData("ekfY", ekf.y());
                telemetry.addData("ekfHeadingDeg", ekf.heading() / MathUtils.DEG_TO_RAD);
                telemetry.addData("rawX", raw.x());
                telemetry.addData("rawY", raw.y());
                telemetry.addData("rawHeadingDeg", raw.heading() / MathUtils.DEG_TO_RAD);
                telemetry.addData("innovX", raw.x() - ekf.x());
                telemetry.addData("innovY", raw.y() - ekf.y());
                telemetry.addData("innovHeadingDeg",
                        Angle.normalizeSigned(raw.heading() - ekf.heading()) / MathUtils.DEG_TO_RAD);
                telemetry.addData("covTrace", covTrace);
                telemetry.addData("covX", covX);
                telemetry.addData("covY", covY);
                telemetry.addData("covHeading", covHeading);
                telemetry.update();
            }
        } finally {
            follower.stop();
        }
    }
}
