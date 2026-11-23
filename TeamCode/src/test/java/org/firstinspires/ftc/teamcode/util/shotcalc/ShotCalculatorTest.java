package org.firstinspires.ftc.teamcode.util.shotcalc;

import static org.junit.Assert.assertEquals;

import com.pedropathing.math.Velocity;

import com.rmsftc.utils.AllianceColor;
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

        // Distance to the red untipped goal (60, 60) is 40 in. Main table
        // segment (36.9, 800) -> (49.5, 820): ratio = 3.1/12.6, smoothstep
        // 3t^2 - 2t^3, so 800 + smoothstep(3.1/12.6) * 20.
        assertEquals(803.0361891964311, solution.flywheelTicksPerSecond, EPSILON);
        // CounterRoller table has its own points; 40 in is exactly its
        // second entry (40, 940).
        assertEquals(940.0, solution.counterRollerTicksPerSecond, EPSILON);
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

        // Release pose is (20 + 100 * 0.05, 60) = (25, 60), 35 in from the
        // goal — inside the first table distance (36.9), so the shot clamps
        // to the first calibrated speed, 800. Without compensation the
        // distance would be 40 in and interpolate to ~803.04.
        assertEquals(800.0, solution.flywheelTicksPerSecond, EPSILON);
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
        return context;
    }
}

