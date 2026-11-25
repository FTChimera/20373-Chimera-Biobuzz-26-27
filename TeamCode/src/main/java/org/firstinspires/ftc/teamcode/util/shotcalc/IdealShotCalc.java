package org.firstinspires.ftc.teamcode.util.shotcalc;

import org.firstinspires.ftc.teamcode.planners.Constants;

/**
 * Constant-angle projectile model (Desmos form, consistent inches) for the
 * ideal shot: fixed hood angle, solve the wheel speed for the distance.
 * Replaces the old linear {@code 200 + 20 * distance} model, which matched
 * the table's slope but had no physics and diverged outside the table —
 * exactly where the ideal is used (extrapolation).
 *
 * <p>Calibration: with the BIOBUZZ hive opening center (59.55 in) the model
 * fits the 3-point main table within ~0.5% over the fit family
 * (c = 60–69 deg, h0 = 7–25 in, r = 1.05–1.2 in); the Desmos angle 62.7 deg
 * sits inside that range. See {@link ProjectileShotCalc} for the model and
 * {@code shotcalc-model-draft/README.md} for the derivation and the fit.
 *
 * <p>Shoot-on-the-move: the wheel speed is compensated for robot velocity
 * ({@link ProjectileShotCalc#compensateTicks}) on top of the latency-based
 * release pose. A shot that cannot be made — too close for the fixed angle,
 * or the robot already moving faster than the required launch speed — degrades
 * to 0 ticks ("do not shoot") instead of throwing: the shooter loop calls
 * this with no guard, and its up-to-speed gate is false at a zero target.
 */
public class IdealShotCalc implements ShotCalculator {
    @Override
    public ShotSolution calculateShot(ShotContext context) {
        if (context == null) {
            throw new IllegalArgumentException("Shot context is required.");
        }
        ShotContext ctx = context.copy();
        ctx.validate();
        if (ctx.sotm) {
            ctx.robotPose = ctx.poseAtRelease();
        }

        double distance = ctx.distanceToTarget();
        double launchSpeed = ProjectileShotCalc.launchSpeed(
                distance,
                Constants.Shooter.ShotCalc.LAUNCH_ANGLE_DEG,
                Constants.Shooter.ShotCalc.LAUNCH_HEIGHT_IN,
                Constants.Shooter.ShotCalc.GOAL_HEIGHT_IN);
        if (!Double.isFinite(launchSpeed)) {
            // Too close for the fixed launch angle: no speed reaches the goal.
            return new ShotSolution(0.0, 0.0);
        }
        double mainTicks = ProjectileShotCalc.ticksPerSecond(launchSpeed, Constants.Shooter.ShotCalc.WHEEL_RADIUS_IN);
        double counterTicks = mainTicks * Constants.Shooter.ShotCalc.COUNTER_ROLLER_RATIO;
        if (ctx.sotm && ctx.velocity != null) {
            double angleToGoal = ctx.angleToTarget();
            double shotDirX = Math.cos(angleToGoal);
            double shotDirY = Math.sin(angleToGoal);
            mainTicks = ProjectileShotCalc.compensateTicks(
                    mainTicks, ctx.velocity.vx, ctx.velocity.vy, shotDirX, shotDirY);
            counterTicks = ProjectileShotCalc.compensateTicks(
                    counterTicks, ctx.velocity.vx, ctx.velocity.vy, shotDirX, shotDirY);
        }
        return new ShotSolution(mainTicks, counterTicks);
    }
}
