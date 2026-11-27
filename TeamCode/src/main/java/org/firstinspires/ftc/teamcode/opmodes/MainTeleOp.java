package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.rmsftc.ftc.ArchitecturalOpMode;
import com.rmsftc.utils.RobotMode;
import com.rmsftc.utils.config.vars.Var;

import org.firstinspires.ftc.teamcode.config.OpModeStorage;
import org.firstinspires.ftc.teamcode.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.Robot;
import org.firstinspires.ftc.teamcode.planners.Constants;
import org.firstinspires.ftc.teamcode.util.CompTeleNames;
import org.firstinspires.ftc.teamcode.util.dashboard.DashboardVars;
import com.rmsftc.utils.DriveVector;

/**
 * Competition TeleOp. One loop, top to bottom: sample the controls once,
 * then intake, parking, drive, and outtake in that order. Parking is plain
 * state on this class, not a scheduled command: the robot drives the park
 * path through the follower, and this loop retries it every half second
 * until it starts and watches for it to finish.
 *
 * <p>Controls are sampled once per active loop, before arbitration and
 * robot updates; held overrides never depend on trigger edges.
 *
 * <p>The outtake arbitration lives inline in {@link #outtakeLoop()}: shooter
 * priority is REVERSE &gt; PANIC &gt; SMART &gt; OFF, feed follows the shooter
 * mode with eject, auto-fire, slow, and the latched manual fast burst.
 */
@TeleOp(name = CompTeleNames.TWO_DRIVERS, group = org.firstinspires.ftc.teamcode.pedro.Constants.COMPETITION_GROUP)
public class MainTeleOp extends ArchitecturalOpMode {
    public static final String NAME = CompTeleNames.ONE_DRIVER;
    public static final Var<Double> AUTO_FIRE_HEADING_TOLERANCE_DEGREES =
            DashboardVars.doubleVar("TeleOp auto-fire heading tolerance (deg)", Constants.TeleOp.AutoFire.HEADING_TOLERANCE_DEG_DEFAULT, Constants.TeleOp.AutoFire.MIN, Constants.TeleOp.AutoFire.MAX);

    private enum ShooterMode { OFF, SMART, PANIC, REVERSE }
    private enum FeedMode { STOP, FAST, SLOW, PANIC, EJECT }

    private Controls controls;
    private Robot robot;
    private ShooterMode shooterMode = ShooterMode.OFF;
    private FeedMode feedMode = FeedMode.STOP;
    private boolean manualFastStarted;
    private boolean parkingInitiated;
    private boolean parkingRequested;
    private boolean automaticParkingCancelled;
    private double lastParkAttemptTimeSec = Double.NEGATIVE_INFINITY;
    private boolean started;
    private boolean cleanedUp;

    protected boolean oneController = false;

    @Override
    protected void onInit() {
        // Stored config applies as-is (or Storage defaults when nothing was
        // ever saved — that is the "TeleOp first" case). Nothing here resets
        // it: resetToDefault() persists, so calling it on init silently
        // clobbered a saved RED alliance to BLUE. The explicit reset lives
        // in ConfigOpMode.
        robot = createRobot();
        robot.matchMode = OpModeStorage.matchMode.get();
        robot.debug = true;
        robot.setManualBrake(false);
        robot.setSmartIntakeControl(true);
        robot.setSmartShooterControl(false);
        DashboardVars.upload(AUTO_FIRE_HEADING_TOLERANCE_DEGREES);
        controls = new Controls(gamepad1, gamepad2, oneController);
    }

    protected Robot createRobot() {
        return new Robot(telemetry, hardwareMap, OpModeStorage.allianceColor.get(),
                RobotMode.TELEOP, OpModeStorage.pose.get());
    }

    @Override
    protected void onLoop() {
        if (cleanedUp) return;
        // Start FIRST: startNanos is 0 until start(), so opModeTimeRemaining()
        // reads deeply negative on tick 1 if the timer check runs before this
        // (Clock.SYSTEM is nanoTime, monotonic since boot — any device up
        // more than ~2 minutes ends the OpMode before start() ever runs).
        if (!started) {
            robot.start();
            started = true;
        }
        if (robot.opModeTimeRemaining() <= 0) {
            cleanup();
            super.interrupt();
            return;
        }
        controls.update();
        intakeLoop();
        parkingLoop();
        driveLoop();
        outtakeLoop();
        robot.telemetry.addData("Parking", "requested=%s started=%s auto cancelled=%s",
                parkingRequested, parkingInitiated, automaticParkingCancelled);
        robot.update();
    }

    private void intakeLoop() {
        if (controls.intakeControl().risingEdge()) {
            robot.setSmartIntakeControl(true);
            robot.lights.onIntakeOn();
        } else if (controls.intakeControl().fallingEdge()) {
            robot.setSmartIntakeControl(false);
            robot.intake.stop();
            robot.lights.onIntakeOff();
        }
        if (controls.deployer().risingEdge() || controls.deployer().fallingEdge()) {
            boolean deployed = controls.deployer().isPressed();
            robot.intake.setDeployed(deployed);
            robot.lights.onDeployerToggle(deployed);
        }
        if (controls.intakeEject().risingEdge()) {
            robot.intakeEject();
            robot.lights.onEjectStart();
        } else if (controls.intakeEject().fallingEdge()) {
            robot.restoreIntake(controls.intakeControl().isPressed()
                    ? Intake.Mode.FULL_POWER : Intake.Mode.IDLE);
        }
        if (controls.reZeroGarden().isPressed()) {
            if (controls.reZeroGarden().risingEdge()) robot.lights.onReZero();
            robot.reZeroGarden();
        }
        if (controls.reZeroLoading().isPressed()) {
            if (controls.reZeroLoading().risingEdge()) robot.lights.onReZero();
            robot.reZeroCornerNextToLoadingZone();
        }
    }

    private void parkingLoop() {
        // DOWN wins simultaneous UP/DOWN and suppresses automatic retries.
        if (controls.stopPark().isPressed()) {
            if (controls.stopPark().risingEdge()) cancelParking();
            return;
        }
        if (controls.moveToParkZone().risingEdge()) {
            parkingRequested = true;
            parkingInitiated = false;
            automaticParkingCancelled = false;
        } else if (!automaticParkingCancelled && !parkingInitiated && robot.shouldPark()) {
            parkingRequested = true;
        }
        if (parkingInitiated && !robot.follower.following()) {
            // Journey finished (or already parked): stay holding the
            // endpoint, and don't re-request until the driver asks again.
            parkingRequested = false;
        }
        if (parkingRequested && !parkingInitiated) {
            double now = robot.getOpModeTimeSeconds();
            if (now - lastParkAttemptTimeSec >= Constants.Robot.Park.RETRY_INTERVAL_SEC) {
                lastParkAttemptTimeSec = now;
                parkingInitiated = robot.moveToParkZone();
                if (parkingInitiated) robot.lights.onParkStart();
            }
        }
    }

    private void cancelParking() {
        robot.cancelPark();
        parkingRequested = false;
        parkingInitiated = false;
        automaticParkingCancelled = true;
    }

    private void driveLoop() {
        DriveVector drive = new DriveVector(-controls.primary().left_stick_y,
                -controls.primary().left_stick_x, -controls.primary().right_stick_x);
        boolean aligning = controls.autoAlignAndStartShoot().isPressed();
        // Robot ignores drive during FOLLOW, but currently accepts it in HOLD.
        // Preserve the actual endpoint, not a new hold at the drifting pose.
        boolean parkHold = parkingInitiated && robot.follower.holding();
        boolean neutral = !robot.follower.hasInput(drive, Constants.Robot.Drive.INPUT_DEADBAND);
        if (!parkHold || !neutral || aligning) {
            if (parkHold && (!neutral || aligning)) cancelParking();
            robot.setTeleOpDrive(drive, true, false);
        }
        if (aligning) robot.setAutoAlign();
    }

    private void outtakeLoop() {
        ShooterMode previous = shooterMode;
        boolean reverse = controls.reverseOuttake().isPressed();
        boolean panic = controls.panicShoot().isPressed();
        boolean shooterEnabled = controls.keepShooterOn().isPressed()
                || controls.autoAlignAndStartShoot().isPressed();
        boolean fast = controls.runTransferFast().isPressed();
        boolean slow = controls.runTransferSlow().isPressed();
        boolean autoFire = controls.autoAlignAndStartShoot().isPressed();
        boolean eject = controls.intakeEject().isPressed();
        boolean upToSpeed = robot.shooter.upToSpeed();
        boolean headingReady = headingReady(robot.autoAimerCalculator.getTargetHeading(),
                robot.getHeading(), AUTO_FIRE_HEADING_TOLERANCE_DEGREES.get());

        shooterMode = reverse ? ShooterMode.REVERSE
                : panic ? ShooterMode.PANIC
                : shooterEnabled ? ShooterMode.SMART : ShooterMode.OFF;
        // RT is a manual burst: speed-gated on entry, not interrupted by each
        // ball's speed dip. Explicit shutdown or an override ends that burst.
        if (!fast || shooterMode != ShooterMode.SMART || eject
                || previous != ShooterMode.SMART) {
            manualFastStarted = false;
        }
        if (fast && shooterMode == ShooterMode.SMART && !eject
                && previous == ShooterMode.SMART && upToSpeed) {
            manualFastStarted = true;
        }

        if (shooterMode == ShooterMode.REVERSE) feedMode = FeedMode.STOP;
        else if (shooterMode == ShooterMode.PANIC) feedMode = FeedMode.PANIC;
        else if (eject) feedMode = FeedMode.EJECT;
        else if (shooterMode == ShooterMode.SMART && previous == ShooterMode.SMART
                && autoFire && upToSpeed && headingReady) feedMode = FeedMode.FAST;
        else if (slow) feedMode = FeedMode.SLOW;
        else if (manualFastStarted) feedMode = FeedMode.FAST;
        else feedMode = FeedMode.STOP;

        if (previous != shooterMode) {
            // End old overrides before applying the winner; their end methods
            // stop both shooter and transfer and must never run after it.
            if (previous == ShooterMode.REVERSE) robot.endReverseOuttake();
            if (previous == ShooterMode.PANIC) robot.endPanicShoot();
        }
        robot.setSmartShooterControl(shooterMode == ShooterMode.SMART);
        switch (shooterMode) {
            case REVERSE:
                robot.reverseOuttake();
                if (previous != shooterMode) robot.lights.onReverseOuttakeStart();
                break;
            case PANIC:
                robot.panic();
                if (previous != shooterMode) robot.lights.onPanicStart();
                break;
            case OFF:
                robot.shooter.setVelocity(0, 0);
                break;
            case SMART:
                if (previous != shooterMode) robot.lights.onShootStart();
                break;
        }
        switch (feedMode) {
            case FAST: robot.transfer.shootAll(); break;
            case SLOW: robot.transfer.slow(); break;
            case PANIC: robot.transfer.panicShoot(); break;
            case EJECT: robot.transfer.reverse(); break;
            case STOP: robot.transfer.stop(); break;
        }
    }

    /** Wrap-safe heading check for auto-fire: true when the heading error is within tolerance. */
    static boolean headingReady(double target, double heading, double toleranceDegrees) {
        if (!Double.isFinite(toleranceDegrees) || toleranceDegrees < 0
                || toleranceDegrees > 180) {
            throw new IllegalArgumentException("Heading tolerance must be between 0 and 180 degrees.");
        }
        if (!Double.isFinite(target) || !Double.isFinite(heading)) return false;
        double error = Math.atan2(Math.sin(target - heading), Math.cos(target - heading));
        return Math.abs(error) <= Math.toRadians(toleranceDegrees);
    }

    private void cleanup() {
        if (cleanedUp) return;
        cleanedUp = true;
        try {
            if (robot != null) {
                robot.setSmartIntakeControl(false);
                robot.setSmartShooterControl(false);
                robot.setTeleOpDrive = false;
                try {
                    cancelParking();
                } finally {
                    robot.stopEverything();
                }
            }
        } finally {
            OpModeStorage.endMatch();
        }
    }

    @Override
    public void onStop() {
        try {
            cleanup();
        } finally {
            super.onStop();
        }
    }

    @TeleOp(name = CompTeleNames.ONE_DRIVER, group = org.firstinspires.ftc.teamcode.pedro.Constants.COMPETITION_GROUP)
    public static class MainTeleOneController extends MainTeleOp {
        public MainTeleOneController() {
            this.oneController = true;
        }
    }
}
