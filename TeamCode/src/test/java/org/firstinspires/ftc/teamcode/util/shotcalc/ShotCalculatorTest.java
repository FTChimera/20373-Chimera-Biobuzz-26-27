package org.firstinspires.ftc.teamcode.util.shotcalc;

import static org.junit.Assert.assertEquals;

import com.pedropathing.math.Velocity;

import org.firstinspires.ftc.teamcode.util.AllianceColor;
import org.firstinspires.ftc.teamcode.util.game.BallCol;
import org.firstinspires.ftc.teamcode.util.game.Field;
import org.firstinspires.ftc.teamcode.util.poses.Pose2d;
import org.junit.Test;

public class ShotCalculatorTest {
    private static final double EPSILON = 1e-9;

    @Test
    public void calculatesTargetDistanceAndHeading() {
        ShotContext context = pollenContext(new Pose2d(30, 60, 0));

        assertEquals(30.0, context.distanceToTarget(), EPSILON);
        assertEquals(0.0, context.angleToTarget(), EPSILON);
    }

    @Test
    public void interpolatesPollenShotInEncoderTicksPerSecond() {
        ShotContext context = pollenContext(new Pose2d(20, 60, 0));

        ShotSolution solution = ShotCalculator.table().calculateShot(context);

        assertEquals(1000.0, solution.flywheelTicksPerSecond, EPSILON);
    }

    @Test
    public void idealCalculatorMatchesDefaultTableSlope() {
        ShotContext context = pollenContext(new Pose2d(20, 60, 0));

        ShotSolution solution = ShotCalculator.ideal().calculateShot(context);

        assertEquals(1000.0, solution.flywheelTicksPerSecond, EPSILON);
    }

    @Test
    public void compensatesShotOnMoveOnlyAcrossLoopLatency() {
        ShotContext context = pollenContext(new Pose2d(20, 60, 0));
        context.sotm = true;
        context.dt = 0.05;
        context.velocity = new Velocity(100, 0, 0);

        ShotSolution solution = ShotCalculator.table().calculateShot(context);

        assertEquals(900.0, solution.flywheelTicksPerSecond, EPSILON);
    }

    @Test
    public void clampsCloseShotsToFirstCalibratedSpeed() {
        ShotContext context = pollenContext(new Pose2d(50, 60, 0));

        ShotSolution solution = ShotCalculator.table().calculateShot(context);

        assertEquals(800.0, solution.flywheelTicksPerSecond, EPSILON);
    }

    private ShotContext pollenContext(Pose2d robotPose) {
        ShotContext context = new ShotContext();
        context.robotPose = robotPose;
        context.dt = 0.0;
        context.hiveState = Field.HiveState.UNTIPPED;
        context.allianceColor = AllianceColor.RED;
        context.transferBallCol = BallCol.POLLEN_YELLOW;
        return context;
    }
}
