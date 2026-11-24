package org.firstinspires.ftc.teamcode.util.shotcalc;

import static org.junit.Assert.assertEquals;

import com.pedropathing.math.Velocity;

import com.rmsftc.utils.AllianceColor;
import org.firstinspires.ftc.teamcode.util.game.Field;
import com.rmsftc.utils.math.Pose;
import org.junit.Test;

public class ShotCalculatorTest {
    private static final double EPSILON = 1e-9;

    @Test
    public void calculatesTargetDistanceAndHeading() {
        ShotContext context = pollenContext(new Pose(30, 60, 0));

        assertEquals(30.0, context.distanceToTarget(), EPSILON);
        assertEquals(0.0, context.angleToTarget(), EPSILON);
    }

    @Test
    public void interpolatesPollenShotInEncoderTicksPerSecond() {
        ShotContext context = pollenContext(new Pose(20, 60, 0));

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
    public void idealCalculatorUsesProjectileModel() {
        ShotContext context = pollenContext(new Pose(20, 60, 0));

        ShotSolution solution = ShotCalculator.ideal().calculateShot(context);

        // Distance to the red untipped goal (60, 60) is 40 in. Fixed-angle
        // projectile (c = 62.7 deg, h0 = 19 in, H = 59.55 in, r = 1.1 in):
        // u = sqrt(g d^2 / (2 cos^2(c) (d tan(c) - (H - h0)))) = 199.347 in/s,
        // ticks = u / (2 pi r) * 28 = 807.597. Counter roller is the main
        // speed times COUNTER_ROLLER_RATIO (1.0 until it is re-measured).
        assertEquals(807.5973669097598, solution.flywheelTicksPerSecond, EPSILON);
        assertEquals(807.5973669097598, solution.counterRollerTicksPerSecond, EPSILON);
    }

    @Test
    public void compensatesShotOnMoveForLatencyAndRobotVelocity() {
        ShotContext context = pollenContext(new Pose(20, 60, 0));
        context.sotm = true;
        context.dt = 0.05;
        context.velocity = new Velocity(30, 0, 0);

        ShotSolution solution = ShotCalculator.table().calculateShot(context);

        // Release pose is (20 + 30 * 0.05, 60) = (21.5, 60), 38.5 in from the
        // goal. Main table at 38.5 in interpolates to 800.886 ticks/s; the
        // counter table to 919.345. Driving 30 in/s straight at the goal, the
        // ball inherits that velocity, so the wheels spin slower: the
        // horizontal launch speed needed is Vreq - 30, giving 535.898
        // (main) and 654.357 (counter) ticks/s.
        assertEquals(535.8978685468263, solution.flywheelTicksPerSecond, EPSILON);
        assertEquals(654.3572750384059, solution.counterRollerTicksPerSecond, EPSILON);
    }

    @Test
    public void infeasibleShotOnMoveReturnsZeroVelocity() {
        ShotContext context = pollenContext(new Pose(20, 60, 0));
        context.sotm = true;
        context.dt = 0.05;
        context.velocity = new Velocity(100, 0, 0);

        ShotSolution solution = ShotCalculator.table().calculateShot(context);

        // Release pose is (25, 60), 35 in from the goal. The table speed
        // there (800 ticks/s) needs only ~91 in/s of horizontal launch
        // speed, but the robot is already moving 100 in/s toward the goal —
        // no wheel speed can make that shot. It degrades to 0 ticks
        // ("do not shoot") instead of throwing; the shooter's up-to-speed
        // gate is false at a zero target, so the attempt times out.
        assertEquals(0.0, solution.flywheelTicksPerSecond, EPSILON);
        assertEquals(0.0, solution.counterRollerTicksPerSecond, EPSILON);
    }

    @Test
    public void idealCalculatorReturnsZeroWhenTooCloseForFixedAngle() {
        ShotContext context = pollenContext(new Pose(45, 60, 0));

        ShotSolution solution = ShotCalculator.ideal().calculateShot(context);

        // 15 in from the goal: d * tan(62.7 deg) = 29.1 in < rise 40.55 in,
        // so no launch speed at this fixed angle reaches the goal height.
        // Degrades to 0 ticks instead of throwing.
        assertEquals(0.0, solution.flywheelTicksPerSecond, EPSILON);
        assertEquals(0.0, solution.counterRollerTicksPerSecond, EPSILON);
    }

    @Test
    public void clampsCloseShotsToFirstCalibratedSpeed() {
        ShotContext context = pollenContext(new Pose(50, 60, 0));

        ShotSolution solution = ShotCalculator.table().calculateShot(context);

        assertEquals(800.0, solution.flywheelTicksPerSecond, EPSILON);
    }

    private ShotContext pollenContext(Pose robotPose) {
        ShotContext context = new ShotContext();
        context.robotPose = robotPose;
        context.dt = 0.0;
        context.hiveState = Field.HiveState.UNTIPPED;
        context.allianceColor = AllianceColor.RED;
        return context;
    }
}

