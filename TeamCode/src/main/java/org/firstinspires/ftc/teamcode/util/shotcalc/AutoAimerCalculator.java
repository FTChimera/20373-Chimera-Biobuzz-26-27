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

    public void updatePedro(Pose robotPose, Velocity velocity, double latencySeconds) {
        if (robotPose == null || velocity == null) {
            throw new IllegalArgumentException("Robot pose and velocity are required for Pedro auto-aim.");
        }
        if (!Double.isFinite(latencySeconds) || latencySeconds < 0.0) {
            throw new IllegalArgumentException("Auto-aim latency must be finite and nonnegative.");
        }
        this.pedro = true;
        correctedPose = PoseFactory.of(robotPose).exp(velocity, latencySeconds);
        Pose hivePose = hiveState.goalPose(color);
        targetHeading = Angle.normalize(Math.atan2(
                hivePose.y() - correctedPose.y(),
                hivePose.x() - correctedPose.x()
        ));
        // Shoot-on-the-move: the ball inherits robot velocity, so the aim
        // heading is the compensated launch direction, not the raw bearing.
        // The offset is the same vector equation the shot calculators use
        // (L = Vreq*s - v): L's perpendicular component is -vPerp, so the
        // heading rotates by atan2(-vPerp, launchHorizontal) — against the
        // robot's sideways drift. When the projectile solve is infeasible
        // compensation is infeasible (robot faster than the required launch
        // speed), keep the raw bearing: this runs in the aimer daemon every
        // tick and must never throw. The offset uses the ideal launch speed;
        // the table the shooter runs differs from it by <1%, so the heading
        // error from that is under 0.2 deg.
        double distance = Math.hypot(
                hivePose.x() - correctedPose.x(),
                hivePose.y() - correctedPose.y());
        double launchSpeed = ProjectileShotCalc.launchSpeed(
                distance,
                ProjectileShotCalc.LAUNCH_ANGLE_DEG,
                ProjectileShotCalc.LAUNCH_HEIGHT_IN,
                ProjectileShotCalc.GOAL_HEIGHT_IN);
        if (Double.isFinite(launchSpeed)) {
            double shotDirX = Math.cos(targetHeading);
            double shotDirY = Math.sin(targetHeading);
            double[] compensation = ProjectileShotCalc.compensate(
                    launchSpeed, ProjectileShotCalc.LAUNCH_ANGLE_DEG,
                    velocity.vx, velocity.vy, shotDirX, shotDirY);
            if (compensation[0] > 0.0) {
                targetHeading = Angle.normalize(targetHeading + compensation[1]);
            }
        }
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
            return clamp(turningpower * 0.5);
        }
        // normalizeSigned, not normalize: the error must keep its sign, or a
        // heading above the target reads as a large positive error and the
        // robot turns the wrong way (clamped to full power).
        double error = Angle.normalizeSigned(targetHeading - currentHeading);
        double result = error * 0.5; // TODO: tune on auto align test
        return clamp(result);
    }

    private double clamp(double value) {
        return Math.max(-1.0, Math.min(1.0, value));
    }
}
