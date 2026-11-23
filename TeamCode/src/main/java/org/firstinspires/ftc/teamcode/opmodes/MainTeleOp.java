package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.rmsftc.cmdlib.EndCondition;
import com.rmsftc.cmdlib.Status;
import com.rmsftc.cmdlib.cmd.Command;
import com.rmsftc.cmdlib.cmd.Commands;
import com.rmsftc.ftc.ArchitectureManager;
import com.rmsftc.ftc.ArchitecturalOpMode;
import com.rmsftc.utils.RobotMode;
import com.rmsftc.utils.config.vars.Var;

import org.firstinspires.ftc.teamcode.config.OpModeStorage;
import org.firstinspires.ftc.teamcode.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.Robot;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.util.CompTeleNames;
import org.firstinspires.ftc.teamcode.util.dashboard.DashboardVars;
import com.rmsftc.utils.DriveVector;

/**
 * Competition TeleOp. Controls are sampled once per active loop, before
 * arbitration and robot daemons; held overrides never depend on trigger edges.
 */
@TeleOp(name = CompTeleNames.TWO_DRIVERS, group = Constants.COMPETITION_GROUP)
public class MainTeleOp extends ArchitecturalOpMode {
    public static final String NAME = CompTeleNames.ONE_DRIVER;
    public static final Var<Double> AUTO_FIRE_HEADING_TOLERANCE_DEGREES =
            DashboardVars.doubleVar("TeleOp auto-fire heading tolerance (deg)", 3.0, 0.0, 180.0);
    private static final double PARK_RETRY_INTERVAL_SEC = 0.5;

    private Controls controls;
    private Robot robot;
    private final TeleOpOuttakeState outtake = new TeleOpOuttakeState();
    private boolean parkingInitiated;
    private boolean parkingRequested;
    private boolean automaticParkingCancelled;
    private double lastParkAttemptTimeSec = Double.NEGATIVE_INFINITY;
    private boolean started;
    private boolean cleanedUp;
    private Command parkCommand;

    protected boolean oneController = false;

    /** Owns retries and the journey, not just acceptance of a path request. */
    private class ParkCommand extends Command {
        @Override
        public Status execute() {
            // BA executes a snapshot: cancellation by controlLoop must not
            // restart this command later in that same scheduler tick.
            if (cleanedUp || !manager.scheduler().isScheduled(this)
                    || robot.opModeTimeRemaining() <= 0) return Status.of(false);
            return super.execute();
        }

        @Override
        public void start() {
            parkingRequested = true;
            parkingInitiated = false;
            automaticParkingCancelled = false;
        }

        @Override
        public void loop() {
            if (cleanedUp || !started || parkingInitiated) return;
            double now = robot.getOpModeTimeSeconds();
            if (now - lastParkAttemptTimeSec >= PARK_RETRY_INTERVAL_SEC) {
                lastParkAttemptTimeSec = now;
                parkingInitiated = robot.moveToParkZone();
                if (parkingInitiated) robot.lights.onParkStart();
            }
        }

        @Override
        public Status status() {
            return Status.of(!cleanedUp
                    && (!parkingInitiated || robot.follower.following()));
        }

        @Override
        public void end(EndCondition endCondition) {
            parkingRequested = false;
            if (endCondition == EndCondition.INTERRUPTED) {
                robot.cancelPark();
                parkingInitiated = false;
                automaticParkingCancelled = true;
            }
            // Natural completion leaves the follower's endpoint HOLD intact.
        }
    }

    @Override
    protected void onInit() {
        if (!OpModeStorage.initialized.get()) OpModeStorage.resetToDefault();
        robot = new Robot(telemetry, hardwareMap, OpModeStorage.allianceColor.get(),
                RobotMode.TELEOP, OpModeStorage.pose.get());
        robot.matchMode = OpModeStorage.matchMode.get();
        robot.debug = true;
        robot.setManualBrake(false);
        robot.setSmartIntakeControl(true);
        robot.setSmartShooterControl(false);
        DashboardVars.upload(AUTO_FIRE_HEADING_TOLERANCE_DEGREES);
        controls = new Controls(gamepad1, gamepad2, oneController);
        parkCommand = new ParkCommand();
    }

    @Override
    protected void build(ArchitectureManager manager) {
        manager.schedule(Commands.run(this::controlLoop));
    }

    private void controlLoop() {
        if (cleanedUp) return;
        // BA ticks scheduled commands before onLoop, including the first tick.
        if (!started) {
            robot.start();
            started = true;
        }
        // Defer interrupt/cancelAll to onLoop, outside the scheduler snapshot.
        if (robot.opModeTimeRemaining() <= 0) return;
        controls.update();
        intakeLoop();
        parkingLoop();
        driveLoop();
        outtakeLoop();
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
        boolean manualRequest = controls.moveToParkZone().risingEdge();
        if (!manager.scheduler().isScheduled(parkCommand)
                && (manualRequest || (!automaticParkingCancelled
                && !parkingInitiated && robot.shouldPark()))) {
            manager.schedule(parkCommand);
        }
    }

    private void cancelParking() {
        manager.cancel(parkCommand);
        // A pending, not-yet-executed command has no end callback in BA.
        // This also releases HOLD after the journey completes naturally.
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
        boolean neutral = !robot.follower.hasInput(drive, 0.1);
        if (!parkHold || !neutral || aligning) {
            if (parkHold && (!neutral || aligning)) cancelParking();
            robot.setTeleOpDrive(drive, true, false);
        }
        if (aligning) robot.setAutoAlign();
    }

    private void outtakeLoop() {
        TeleOpOuttakeState.ShooterMode previous = outtake.shooter;
        outtake.update(controls.reverseOuttake().isPressed(), controls.panicShoot().isPressed(),
                controls.keepShooterOn().isPressed()
                        || controls.autoAlignAndStartShoot().isPressed(),
                controls.runTransferFast().isPressed(), controls.runTransferSlow().isPressed(),
                controls.autoAlignAndStartShoot().isPressed(), controls.intakeEject().isPressed(),
                robot.shooter.upToSpeed(),
                TeleOpOuttakeState.headingReady(robot.autoAimerCalculator.getTargetHeading(),
                        robot.getHeading(), AUTO_FIRE_HEADING_TOLERANCE_DEGREES.get()));

        if (previous != outtake.shooter) {
            // End old overrides before applying the winner; their end methods
            // stop both shooter and transfer and must never run after it.
            if (previous == TeleOpOuttakeState.ShooterMode.REVERSE) robot.endReverseOuttake();
            if (previous == TeleOpOuttakeState.ShooterMode.PANIC) robot.endPanicShoot();
        }
        robot.setSmartShooterControl(outtake.shooter == TeleOpOuttakeState.ShooterMode.SMART);
        switch (outtake.shooter) {
            case REVERSE:
                robot.reverseOuttake();
                if (previous != outtake.shooter) robot.lights.onReverseOuttakeStart();
                break;
            case PANIC:
                robot.panic();
                if (previous != outtake.shooter) robot.lights.onPanicStart();
                break;
            case OFF:
                robot.shooter.setVelocity(0, 0);
                break;
            case SMART:
                if (previous != outtake.shooter) robot.lights.onShootStart();
                break;
        }
        switch (outtake.feed) {
            case FAST: robot.transfer.shootAll(); break;
            case SLOW: robot.transfer.slow(); break;
            case PANIC: robot.transfer.panicShoot(); break;
            case EJECT: robot.transfer.reverse(); break;
            case STOP: robot.transfer.stop(); break;
        }
    }

    @Override
    protected void onLoop() {
        if (cleanedUp) return;
        if (robot.opModeTimeRemaining() <= 0) {
            interrupt();
            return;
        }
        robot.telemetry.addData("Parking", "requested=%s started=%s auto cancelled=%s",
                parkingRequested, parkingInitiated, automaticParkingCancelled);
        robot.update();
        robot.telemetry.update();
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

    @Override
    public void interrupt() {
        try {
            cleanup();
        } finally {
            super.interrupt();
        }
    }

    @TeleOp(name = CompTeleNames.ONE_DRIVER, group = Constants.COMPETITION_GROUP)
    public static class MainTeleOneController extends MainTeleOp {
        public MainTeleOneController() {
            this.oneController = true;
        }
    }
}
