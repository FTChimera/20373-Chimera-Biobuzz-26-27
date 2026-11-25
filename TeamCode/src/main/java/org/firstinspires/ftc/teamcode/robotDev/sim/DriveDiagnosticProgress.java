package org.firstinspires.ftc.teamcode.robotDev.sim;

/** Pure local criteria. Inches/radians; time is elapsed seconds since START, not since setPose. */
final class DriveDiagnosticProgress {
    static final double POSITION_TOLERANCE = 2.0;
    static final double HEADING_TOLERANCE = Math.toRadians(8);
    static final double MOVEMENT_REQUIRED = 0.5;
    static final double STALL_MOVEMENT = 0.15;
    static final double STALL_SECONDS = 2.0;
    static final double WHEEL_TICKS_REQUIRED = 12;

    final double timeout;
    private boolean baseline;
    private double lastTime, startX, startY, rawStartX, rawStartY;
    private double stallTime, stallX, stallY;
    private double[] startTicks, stallTicks;
    double poseMovement, pinpointMovement, wheelMovement;
    private String result = "RUNNING";

    DriveDiagnosticProgress(double timeout) {
        if (!Double.isFinite(timeout) || timeout <= 0) {
            throw new IllegalArgumentException("Positive finite timeout required");
        }
        this.timeout = timeout;
    }

    String sample(double time, double x, double y, double heading,
                  double rawX, double rawY, double rawHeading, double[] ticks,
                  double[] powers, double completion, boolean started, boolean completed,
                  boolean atDestination, boolean cancelled) {
        if (finished()) return result;
        if (cancelled) return result = "FAIL: cancelled/STOP";
        if (!finite(time, x, y, heading, rawX, rawY, rawHeading, completion)
                || !validWheels(ticks) || !validWheels(powers)) {
            return result = "FAIL: nonfinite/invalid drive sample";
        }
        if (baseline && time < lastTime) return result = "FAIL: clock moved backwards";
        if (!baseline) {
            baseline = true;
            stallTime = time;
            startX = x;
            startY = y;
            rawStartX = stallX = rawX;
            rawStartY = stallY = rawY;
            startTicks = ticks.clone();
            stallTicks = ticks.clone();
        }
        lastTime = time;
        poseMovement = Math.max(poseMovement, Math.hypot(x - startX, y - startY));
        pinpointMovement = Math.max(pinpointMovement, Math.hypot(rawX - rawStartX, rawY - rawStartY));
        wheelMovement = Math.max(wheelMovement, wheelDelta(ticks, startTicks));
        if (time >= timeout) {
            return result = wheelMovement < WHEEL_TICKS_REQUIRED
                    ? "FAIL: timeout; no independently observed drivetrain movement"
                    : "FAIL: timeout; path did not complete";
        }
        // A rolling raw-sensor window also detects a localizer that dies halfway along the route.
        if (Math.hypot(rawX - stallX, rawY - stallY) >= STALL_MOVEMENT) {
            stallX = rawX;
            stallY = rawY;
            stallTime = time;
            stallTicks = ticks.clone();
        } else if (time - stallTime >= STALL_SECONDS
                && wheelDelta(ticks, stallTicks) >= WHEEL_TICKS_REQUIRED) {
            return result = "FAIL: Pinpoint dead; stop other path diagnostics";
        }
        if (completed) {
            if (!started) return result = "FAIL: completion without a started path";
            if (completion < 1.0) return result = "FAIL: follower exited before full route completion";
            if (!atDestination) return result = "FAIL: follower completed outside destination";
            if (poseMovement < MOVEMENT_REQUIRED || pinpointMovement < MOVEMENT_REQUIRED) {
                return result = "FAIL: completion without live translational Pinpoint movement";
            }
            if (wheelMovement < WHEEL_TICKS_REQUIRED) {
                return result = "FAIL: completion without independently observed drive movement";
            }
            return result = "PASS";
        }
        return result;
    }

    String cancel() {
        if (!finished()) result = "FAIL: cancelled/STOP";
        return result;
    }

    boolean finished() {
        return !result.equals("RUNNING");
    }

    String result() {
        return result;
    }

    static double headingError(double actual, double target) {
        return Math.atan2(Math.sin(target - actual), Math.cos(target - actual));
    }

    static boolean atEndpoint(double x, double y, double heading,
                              double targetX, double targetY, double targetHeading) {
        return finite(x, y, heading, targetX, targetY, targetHeading)
                && Math.hypot(targetX - x, targetY - y) <= POSITION_TOLERANCE
                && Math.abs(headingError(heading, targetHeading)) <= HEADING_TOLERANCE;
    }

    static boolean finite(double... values) {
        for (double value : values) if (!Double.isFinite(value)) return false;
        return true;
    }

    private static boolean validWheels(double[] values) {
        return values != null && values.length == 4 && finite(values);
    }

    private static double wheelDelta(double[] a, double[] b) {
        double max = 0;
        for (int i = 0; i < a.length; i++) max = Math.max(max, Math.abs(a[i] - b[i]));
        return max;
    }
}
