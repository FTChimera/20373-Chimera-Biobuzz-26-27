package org.firstinspires.ftc.teamcode.util.shotcalc;

import static org.firstinspires.ftc.teamcode.util.math.MathUtils.DEG_TO_RAD;


import androidx.annotation.NonNull;

import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import com.pedropathing.utils.Angle;
import com.qualcomm.hardware.limelightvision.LLResult;

import org.firstinspires.ftc.teamcode.util.poses.PoseFactory;
import org.firstinspires.ftc.teamcode.util.AllianceColor;
import org.firstinspires.ftc.teamcode.util.game.Field;
import org.firstinspires.ftc.teamcode.util.poses.Pose2d;

public class AutoAimerCalculator {
    public boolean useFollowerHold;
    public boolean pedro = true;
    Pose2d correctedPose;
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

    public Pose2d getCorrectedPose() {
        return correctedPose;
    }

    public Pose2d getCorrectedPoseOrFallback(Pose fallbackPose) {
        if (correctedPose == null) {
            initializeCorrectedPose(fallbackPose);
        }
        return correctedPose;
    }

    @NonNull
    public String getDebugString() {return "namaste. Your mode is "+ (pedro ? "Pedro Heading" : "Limelight") +" \nyour target heading is "+ getTargetHeading()/DEG_TO_RAD;}

    public double calculateTurningPower(double ehiuafhiushfuidshfsudhfdsufdshfudshfudhfHeading) {
        if (!pedro) {
            return clamp(turningpower * 0.5);
        }
        double error = Angle.normalize(targetHeading - ehiuafhiushfuidshfsudhfdsufdshfudshfudhfHeading);
        double result = error * 0.5; // TODO: tune on auto align test
        return clamp(result);
    }

    private double clamp(double value) {
        return Math.max(-1.0, Math.min(1.0, value));
    }
}
