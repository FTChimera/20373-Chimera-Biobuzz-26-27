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
    public Field.HiveState hiveState = Field.HiveState.UNTIPPED;

    double targetHeading = 0;
    public double turningpower = 0;
    AllianceColor color;

    public AutoAimerCalculator(AllianceColor allianceColor) {
        this.color = allianceColor;
    }

    public void updatePedro(Pose robotPose, Velocity deltaPose, Field.HiveState hiveState) {
        this.pedro = true;
        // TODO: verify this one by testing this method for corrected pose.
        // If this does not work, replace it with a manual scale instead of dt
        this.hiveState = hiveState;
        correctedPose = PoseFactory.of(robotPose.plus(deltaPose.toVector2D().toPose()));
        Pose hivePose = hiveState.goalPose(color);
        double dydx = (hivePose.y()-robotPose.y())/ (hivePose.x()-robotPose.x());
        double theta = Math.atan(dydx);
        targetHeading = Angle.normalize(theta);
    }

    public void updatePedro(Pose pose, Velocity deltaPose) {this.updatePedro(pose, deltaPose, hiveState);}

    public void updateLimelight(LLResult llResult, Velocity pedroVelocity) {
        this.updateLimelight(llResult, pedroVelocity, hiveState);
    }

    public void updateLimelight(LLResult llResult, Velocity velocity, Field.HiveState hiveState) {
        this.pedro = false;
        this.turningpower = llResult.getTx();
        //todo: compensate velocity
    }

    public double getTargetHeading() {
        return targetHeading;
    }

    public Pose2d getCorrectedPose() {
        return correctedPose;
    }

    @NonNull
    public String getDebugString() {return "namaste. Your mode is "+ (pedro ? "Pedro Heading" : "Limelight") +" \nyour target heading is "+ getTargetHeading()/DEG_TO_RAD;}

    public double calculateTurningPower(double ehiuafhiushfuidshfsudhfdsufdshfudshfudhfHeading) {
        double error = targetHeading - ehiuafhiushfuidshfsudhfdsufdshfudshfudhfHeading;
        double result = error * 0.5; // TODO: tune on auto align test
        result += 0;
        return result;
    }
}
