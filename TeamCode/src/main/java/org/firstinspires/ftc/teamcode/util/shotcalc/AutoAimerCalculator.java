package org.firstinspires.ftc.teamcode.util.shotcalc;

import static org.firstinspires.ftc.teamcode.util.math.MathUtils.DEG_TO_RAD;


import androidx.annotation.NonNull;

import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import com.pedropathing.utils.Angle;
import com.qualcomm.hardware.limelightvision.LLResult;

import org.firstinspires.ftc.teamcode.util.poses.PoseFactory;
import com.rmsftc.utils.AllianceColor;
import org.firstinspires.ftc.teamcode.util.game.Field;
import org.firstinspires.ftc.teamcode.planners.Constants;

public class AutoAimerCalculator {
    public boolean useFollowerHold;
    public boolean pedro = true;
    com.rmsftc.utils.math.Pose correctedPose;
    public Field.HiveState hiveState;

    double targetHeading = 0;
    public double turningpower = 0;
    AllianceColor color;

    public AutoAimerCalculator(AllianceColor allianceColor) {
        this.color = allianceColor;
        this.hiveState = Field.HiveState.UNTIPPED;
    }

    /**
     * Pedro auto-aim with the shoot-on-the-move flag. When {@code sotm} is
     * false the robot is treated as stationary: the heading is the raw
     * bearing from the current pose, with no latency pose prediction and no
     * velocity compensation. When true, the pose is predicted forward by
     * the latency and the heading is rotated against the robot's sideways
     * drift (same vector equation the shot calculators use). This is the
     * heading third of the single SOTM master flag ({@code Shooter.useSotm});
     * the other two thirds (wheel-speed compensation, pose-at-release) live
     * in the shot calculators and read the same flag.
     */
    public void updatePedro(Pose robotPose, Velocity velocity, double latencySeconds, boolean sotm) {
        if (robotPose == null || velocity == null) {
            throw new IllegalArgumentException("Robot pose and velocity are required for Pedro auto-aim.");
        }
        if (!Double.isFinite(latencySeconds) || latencySeconds < 0.0) {
            throw new IllegalArgumentException("Auto-aim latency must be finite and nonnegative.");
        }
        this.pedro = true;
        correctedPose = sotm
                ? PoseFactory.of(robotPose).exp(velocity, latencySeconds)
                : PoseFactory.of(robotPose);
        Pose hivePose = hiveState.goalPose(color);
        targetHeading = Angle.normalize(Math.atan2(
                hivePose.y() - correctedPose.y(),
                hivePose.x() - correctedPose.x()
        ));
        if (!sotm) {
            return;
        }
        // Shoot-on-the-move: the ball inherits robot velocity, so the aim
        // heading is the compensated launch direction, not the raw bearing.
        // The offset is the same vector equation the shot calculators use
        // (L = Vreq*s - v): L's perpendicular component is -vPerp, so the
        // heading rotates by atan2(-vPerp, launchHorizontal) — against the
        // robot's sideways drift. When the projectile solve is infeasible
        // compensation is infeasible (robot faster than the required launch
        // speed), keep the raw bearing: this runs in the aimer update every
        // tick and must never throw. The offset uses the ideal launch speed;
        // the table the shooter runs differs from it by <1%, so the heading
        // error from that is under 0.2 deg.
        double distance = Math.hypot(
                hivePose.x() - correctedPose.x(),
                hivePose.y() - correctedPose.y());
        double launchSpeed = ProjectileShotCalc.launchSpeed(
                distance,
                Constants.Shooter.ShotCalc.LAUNCH_ANGLE_DEG,
                Constants.Shooter.ShotCalc.LAUNCH_HEIGHT_IN,
                Constants.Shooter.ShotCalc.GOAL_HEIGHT_IN);
        if (Double.isFinite(launchSpeed)) {
            double shotDirX = Math.cos(targetHeading);
            double shotDirY = Math.sin(targetHeading);
            double[] compensation = ProjectileShotCalc.compensate(
                    launchSpeed, Constants.Shooter.ShotCalc.LAUNCH_ANGLE_DEG,
                    velocity.vx, velocity.vy, shotDirX, shotDirY);
            if (compensation[0] > 0.0) {
                targetHeading = Angle.normalize(targetHeading + compensation[1]);
            }
        }
    }

    /** SOTM off: stationary aim (raw bearing, no pose prediction). */
    public void updatePedro(Pose robotPose, Velocity velocity, double latencySeconds) {
        updatePedro(robotPose, velocity, latencySeconds, false);
    }

    public void updatePedro(Pose robotPose, Velocity velocity) {
        updatePedro(robotPose, velocity, 0.0);
    }

    public void setHiveState(Field.HiveState newHiveState) {
        this.hiveState = newHiveState;
    }
    public void updateLimelight(LLResult llResult, Velocity velocity, double latencySeconds) {
        this.updateLimelight(llResult, velocity, hiveState, latencySeconds);
    }

    public void updateLimelight(LLResult llResult, Velocity velocity, Field.HiveState hiveState) {
        updateLimelight(llResult, velocity, hiveState, 0.0);
    }

    public void updateLimelight(LLResult llResult, Velocity velocity, Field.HiveState hiveState,
                                double latencySeconds) {
        if (llResult == null || velocity == null || hiveState == null) {
            throw new IllegalArgumentException("Limelight result, velocity, and hive state are required for auto-aim.");
        }
        if (!Double.isFinite(latencySeconds) || latencySeconds < 0.0) {
            throw new IllegalArgumentException("Auto-aim latency must be finite and nonnegative.");
        }
        this.pedro = false;
        this.hiveState = hiveState;
        this.turningpower = Math.toRadians(llResult.getTx()) - velocity.omega * latencySeconds;
    }

    public double getTargetHeading() {
        return targetHeading;
    }

    public void initializeCorrectedPose(Pose robotPose) {
        if (robotPose == null) {
            throw new IllegalArgumentException("Robot pose is required to initialize corrected pose.");
        }
        correctedPose = PoseFactory.of(robotPose);
    }

    public com.rmsftc.utils.math.Pose getCorrectedPose() {
        return correctedPose;
    }

    public com.rmsftc.utils.math.Pose getCorrectedPoseOrFallback(Pose fallbackPose) {
        if (correctedPose == null) {
            initializeCorrectedPose(fallbackPose);
        }
        return correctedPose;
    }

    @NonNull
    public String getDebugString() {return "namaste. Your mode is "+ (pedro ? "Pedro Heading" : "Limelight") +" \nyour target heading is "+ getTargetHeading()/DEG_TO_RAD;}

    public double calculateTurningPower(double currentHeading) {
        if (!pedro) {
            return clamp(turningpower * Constants.Shooter.Aimer.TURNING_POWER_GAIN);
        }
        // normalizeSigned, not normalize: the error must keep its sign, or a
        // heading above the target reads as a large positive error and the
        // robot turns the wrong way (clamped to full power).
        double error = Angle.normalizeSigned(targetHeading - currentHeading);
        double result = error * Constants.Shooter.Aimer.TURNING_POWER_GAIN; // TODO: tune on auto align test
        return clamp(result);
    }

    private double clamp(double value) {
        return Math.max(-Constants.Shooter.Aimer.TURNING_POWER_MAX, Math.min(Constants.Shooter.Aimer.TURNING_POWER_MAX, value));
    }
}
