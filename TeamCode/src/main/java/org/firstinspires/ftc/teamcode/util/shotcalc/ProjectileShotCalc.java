package org.firstinspires.ftc.teamcode.util.shotcalc;

/**
 * Constant-angle projectile model shared by the ideal and table shot paths
 * (Desmos form, consistent inches) + robot-velocity compensation.
 *
 * <p>Model: fixed launch angle {@code c}, launch height {@code h0}, goal height
 * {@code h1} (BIOBUZZ hive opening center, 59.55 in above the tiles — manual
 * Fig 9-10: opening 53.5–65.6 in), horizontal distance {@code d},
 * {@code g = 386.09 in/s^2}. The ball must pass through the goal, so
 * {@code u^2 = g d^2 / (2 cos^2(c) (d tan(c) - (h1 - h0)))} and
 * {@code tof = d / (u cos(c))}. Wheel ticks/s = {@code u / (2 pi r) * 28}.
 *
 * <p>The Desmos model this comes from is Shlok's own (co-authored with ChatGPT).
 * Its angle 62.7 deg sits inside the fit family (c = 60–69 deg, h0 = 7–25 in,
 * r = 1.05–1.2 in, max error ~0.5% against the 3-point main table). The same
 * formula with the entry angle fixed instead of the launch angle is what 11329
 * ICE Robotics runs as PHYSICAL and what 23435 Gyrobotic Droids derives in
 * "Hood Angle and Velocity Calcs" — three teams, one equation.
 *
 * <p>Velocity compensation: the ball leaves with the wheel's launch velocity
 * <i>plus</i> the robot's velocity. Let {@code Vreq = u cos(c)} along the shot
 * direction {@code s} (unit vector from the release pose to the goal). The launch
 * vector the wheel must produce is {@code L = Vreq * s - v_robot}: {@code |L|}
 * sets the wheel speed, {@code atan2(L)} sets the aim heading. The parallel
 * component of robot velocity changes the speed; the perpendicular component
 * rotates the heading. This is the exact vector form of 23435's radial /
 * tangential decomposition (their turret offset is the same atan2) and of
 * 11329's SOTF parallel/perp split, without 23435's fixed-flight-time and
 * frozen-vertical-speed approximations.
 *
 * <p>Degrade, don't throw: {@link #launchSpeed} returns {@code NaN} when the
 * fixed angle cannot reach the goal from this distance (too close — the rise
 * exceeds {@code d tan(c)}), and the compensation returns a non-positive
 * launch speed when the robot is already moving faster than the ball needs to
 * go. Callers turn those into "do not shoot" (0 ticks) or keep the raw
 * bearing — never an exception. The shooter loop and the aimer daemon call
 * into here with no guard, and the shooter's up-to-speed gate is false at a
 * zero target, so an infeasible shot times out instead of firing a wrong ball
 * or killing the OpMode.
 */
public final class ProjectileShotCalc {
    private ProjectileShotCalc() {
    }

    public static final double GRAVITY_IN_PER_S2 = 386.09;

    // MEASURE — replace with the robot's real numbers before trusting this.
    // Starting point from the table fit (see shotcalc-model-draft/README.md):
    // (62.7 deg, 19 in, 1.1 in) with H = 59.55 in.
    public static final double LAUNCH_ANGLE_DEG = 62.7;   // MEASURE: fixed hood angle (Desmos angle; fit 60-69 deg)
    public static final double LAUNCH_HEIGHT_IN = 19.0;   // MEASURE: ball release height (fit 7-25 in)
    public static final double GOAL_HEIGHT_IN = 59.55;    // BIOBUZZ hive opening center, manual Fig 9-10 (53.5-65.6 in)
    public static final double WHEEL_RADIUS_IN = 1.1;     // MEASURE: flywheel radius (fit 1.05-1.2 in)
    public static final int TICKS_PER_REV = 28;           // SmartFlywheel.Params(28, 1)
    // TODO: tune — the counter-roller table is non-monotonic (940 at 40 in,
    // 880 at 69.5 in) and needs re-measuring before the ratio means anything.
    // 23435's flywheel calibration (range inversion + line fit) is a concrete
    // method for pinning it.
    public static final double COUNTER_ROLLER_RATIO = 1.0;

    /**
     * Launch speed (in/s) for a stationary shot at horizontal distance
     * {@code d}. Returns {@code NaN} when the fixed angle cannot reach the
     * goal from this distance (needs {@code d * tan(c) > h1 - h0}).
     */
    public static double launchSpeed(double distance, double angleDeg, double launchHeight, double goalHeight) {
        double c = Math.toRadians(angleDeg);
        double rise = goalHeight - launchHeight;
        double denom = 2.0 * Math.cos(c) * Math.cos(c) * (distance * Math.tan(c) - rise);
        if (!(denom > 0.0)) {
            return Double.NaN;
        }
        return Math.sqrt(GRAVITY_IN_PER_S2 * distance * distance / denom);
    }

    /** Projectile flight time (s) for a shot at horizontal distance {@code d}. */
    public static double flightTime(double distance, double launchSpeed, double angleDeg) {
        return distance / (launchSpeed * Math.cos(Math.toRadians(angleDeg)));
    }

    public static double ticksPerSecond(double linearSpeedInPerS, double wheelRadiusIn) {
        return linearSpeedInPerS / (2.0 * Math.PI * wheelRadiusIn) * TICKS_PER_REV;
    }

    public static double linearSpeed(double ticksPerSecond, double wheelRadiusIn) {
        return ticksPerSecond / TICKS_PER_REV * 2.0 * Math.PI * wheelRadiusIn;
    }

    /**
     * Velocity compensation. Returns {@code {launchHorizontalSpeed (in/s),
     * headingOffsetRad}}. The heading offset is how far the aim heading
     * rotates from the raw bearing to the goal to cancel the robot's sideways
     * drift; add it to the bearing. {@code robotVelX/Y} are field-frame in/s;
     * {@code shotDirX/Y} is the unit vector from the release pose to the goal.
     * The launch horizontal speed is non-positive exactly when the robot is
     * already moving faster than the ball needs to go along the shot
     * direction — that shot cannot be made at any wheel speed.
     *
     * <p>Sign: the wheel must produce {@code L = Vreq * s - v}, whose
     * perpendicular component is {@code -vPerp}, so the heading offset is
     * {@code atan2(-vPerp, launchHorizontal)} — the aim rotates <i>against</i>
     * the robot's sideways drift, not with it. (A {@code +vPerp} here aims
     * with the drift and doubles the miss.) The wheel speed uses the full
     * magnitude {@code |L| = hypot(launchHorizontal, vPerp)}, not just the
     * parallel component — see {@link #compensateTicks}.
     */
    public static double[] compensate(double launchSpeed, double angleDeg,
                                      double robotVelX, double robotVelY,
                                      double shotDirX, double shotDirY) {
        double horizontal = launchSpeed * Math.cos(Math.toRadians(angleDeg));
        double vParallel = robotVelX * shotDirX + robotVelY * shotDirY;
        double vPerp = -robotVelX * shotDirY + robotVelY * shotDirX;
        double launchHorizontal = horizontal - vParallel;
        double headingOffset = Math.atan2(-vPerp, launchHorizontal);
        return new double[]{launchHorizontal, headingOffset};
    }

    /**
     * Compensates a wheel speed (ticks/s) for robot velocity and returns the
     * compensated wheel speed. The input speed is converted to a launch speed,
     * the robot-velocity vector is subtracted ({@code L = Vreq * s - v}), and
     * the result is converted back. The wheel speed comes from the full
     * magnitude {@code |L| = hypot(launchHorizontal, vPerp)}: the
     * perpendicular component needs wheel speed too, not just the parallel
     * one. Returns 0.0 when the shot is infeasible (robot moving faster
     * than the required launch speed along the shot direction) — the caller
     * treats 0 as "do not shoot".
     */
    public static double compensateTicks(double ticksPerSecond,
                                         double robotVelX, double robotVelY,
                                         double shotDirX, double shotDirY) {
        double launchSpeed = linearSpeed(ticksPerSecond, WHEEL_RADIUS_IN);
        double[] comp = compensate(launchSpeed, LAUNCH_ANGLE_DEG,
                robotVelX, robotVelY, shotDirX, shotDirY);
        if (!(comp[0] > 0.0)) {
            return 0.0;
        }
        double vPerp = -robotVelX * shotDirY + robotVelY * shotDirX;
        double compensatedSpeed = Math.hypot(comp[0], vPerp)
                / Math.cos(Math.toRadians(LAUNCH_ANGLE_DEG));
        return ticksPerSecond(compensatedSpeed, WHEEL_RADIUS_IN);
    }
}
