package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.rmsftc.cmdlib.EndCondition;
import com.rmsftc.cmdlib.Status;
import com.rmsftc.cmdlib.cmd.Command;
import com.rmsftc.cmdlib.cmd.Commands;
import com.rmsftc.cmdlib.trigger.Trigger;
import com.rmsftc.ftc.ArchitectureManager;
import com.rmsftc.ftc.ArchitecturalOpMode;
import com.rmsftc.utils.Clock;

import org.firstinspires.ftc.teamcode.config.OpModeStorage;
import org.firstinspires.ftc.teamcode.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.Robot;
import org.firstinspires.ftc.teamcode.mechanisms.Transfer;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.util.CompTeleNames;
import com.rmsftc.utils.RobotMode;
import org.firstinspires.ftc.teamcode.util.hardware.gamepad.DriveVector;

/**
 * Competition TeleOp on BehaviorArchitectures.
 *
 * <p>Structure (a faithful port of the old manual loop — same behaviors,
 * same edge semantics):
 * <ul>
 *   <li><b>Drive command</b> (scheduled first, runs forever): updates
 *       {@link Controls}, drives from the sticks, applies Y auto-align,
 *       spins the shooter for A/Y, and requests auto-park. It runs first
 *       each tick so the FancyButton state it and the feed command read
 *       is fresh.</li>
 *   <li><b>Feed command</b> (scheduled second, runs forever): the
 *       transfer feed logic — reverse-outtake blocks feeding, B is
 *       panic, triggers/Y feed when the shooter is up to speed.</li>
 *   <li><b>Triggers</b> on the raw gamepad buttons (live reads, no
 *       FancyButton lag): X toggles smart intake, START toggles the
 *       deployer, BACK ejects/restores, left stick button reverses the
 *       outtake, dpad left/right re-zero, dpad up/down park.</li>
 *   <li><b>Park command</b>: retries {@code moveToParkZone()} every
 *       0.5 s until it takes, as the manual loop did.</li>
 * </ul>
 * No requirement tokens in TeleOp: the manual loop had no scheduler
 * conflicts to protect against (last write wins), and tokens would
 * change behavior (e.g. eject interrupting the intake toggle).
 */
@TeleOp(name = CompTeleNames.TWO_DRIVERS, group = Constants.COMPETITION_GROUP)
public class MainTeleOp extends ArchitecturalOpMode {
    public static final String NAME = CompTeleNames.ONE_DRIVER;

    private Controls controls;
    private Robot robot;
    private boolean parkingInitiated;
    private boolean parkingRequested;
    private boolean automaticParkingCancelled;
    private double lastParkAttemptTimeSec;
    private static final double PARK_RETRY_INTERVAL_SEC = 0.5;

    private boolean started;
    /** Intake toggle state, kept by the toggle command's start/end. Starts on. */
    private boolean intakeToggleOn = true;
    private boolean intakeOnFirstStart = true;
    private Command parkCommand;

    protected boolean oneController = false;

    /** Retries moveToParkZone() every 0.5 s until it takes. */
    private class ParkCommand extends Command {
        @Override
        public void start() {
            parkingRequested = true;
            automaticParkingCancelled = false;
        }

        @Override
        public void loop() {
            if (!started || parkingInitiated) return;
            double now = robot.getOpModeTimeSeconds();
            if (now - lastParkAttemptTimeSec >= PARK_RETRY_INTERVAL_SEC) {
                lastParkAttemptTimeSec = now;
                parkingInitiated = robot.moveToParkZone();
                if (parkingInitiated) robot.lights.onParkStart();
            }
        }

        @Override
        public Status status() {
            return Status.of(!parkingInitiated);
        }

        @Override
        public void end(EndCondition endCondition) {
        }
    }

    @Override
    protected void onInit() {
        // ROBOT
        if (!OpModeStorage.initialized.get()) OpModeStorage.resetToDefault();
        robot = new Robot(telemetry, hardwareMap, OpModeStorage.allianceColor.get(), RobotMode.TELEOP, OpModeStorage.pose.get());
        robot.matchMode = OpModeStorage.matchMode.get();
        robot.debug = true;
        parkingInitiated = false;
        parkingRequested = false;
        automaticParkingCancelled = false;
        lastParkAttemptTimeSec = Double.NEGATIVE_INFINITY;

        robot.setManualBrake(false);
        robot.setSmartIntakeControl(true); // Manually control via toggle gp
        robot.setSmartShooterControl(false);

        controls = new Controls(gamepad1, gamepad2, oneController);
        parkCommand = new ParkCommand();
    }

    @Override
    protected void build(ArchitectureManager manager) {
        // Drive: scheduled first so controls.update() runs before the
        // feed command reads button state each tick.
        manager.schedule(Commands.run(this::driveLoop));
        // Feed: the transfer logic, after drive.
        manager.schedule(Commands.run(this::feedLoop));

        // Intake toggle (X): starts ON (scheduled below, matching the
        // toggle button's initial pressed state). Start/end are the old
        // rising/falling edge actions; the first start skips the lights
        // hook, as the old code had no edge at init.
        Command intakeOnCommand = Commands.startEnd(
                () -> {
                    intakeToggleOn = true;
                    robot.setSmartIntakeControl(true);
                    if (!intakeOnFirstStart) robot.lights.onIntakeOn();
                    intakeOnFirstStart = false;
                },
                () -> {
                    intakeToggleOn = false;
                    robot.setSmartIntakeControl(false);
                    robot.intake.stop();
                    robot.lights.onIntakeOff();
                });
        manager.schedule(intakeOnCommand);
        Trigger intakeToggle = new Trigger(() -> controls.secondary().x, manager.scheduler(), Clock.SYSTEM);
        intakeToggle.toggleOnTrue(intakeOnCommand);
        manager.scheduler().addTrigger(intakeToggle);

        // Deployer toggle (START): starts stowed.
        Command deployerCommand = Commands.startEnd(
                () -> {
                    robot.intake.setDeployed(true);
                    robot.lights.onDeployerToggle(true);
                },
                () -> {
                    robot.intake.setDeployed(false);
                    robot.lights.onDeployerToggle(false);
                });
        Trigger deployerToggle = new Trigger(() -> controls.secondary().start, manager.scheduler(), Clock.SYSTEM);
        deployerToggle.toggleOnTrue(deployerCommand);
        manager.scheduler().addTrigger(deployerToggle);

        // Eject (BACK): press ejects, release restores the intake to
        // the mode the toggle button selects.
        Trigger eject = new Trigger(() -> controls.secondary().back, manager.scheduler(), Clock.SYSTEM);
        eject.onTrue(Commands.runOnce(() -> {
            robot.intakeEject();
            robot.lights.onEjectStart();
        }));
        eject.onFalse(Commands.runOnce(() -> robot.restoreIntake(
                intakeToggleOn ? Intake.Mode.FULL_POWER : Intake.Mode.IDLE)));
        manager.scheduler().addTrigger(eject);

        // Reverse outtake (left stick button, held): clear a jam.
        Trigger reverseOuttake = new Trigger(() -> controls.secondary().left_stick_button, manager.scheduler(), Clock.SYSTEM);
        reverseOuttake.whileTrue(Commands.startEnd(
                () -> {
                    robot.reverseOuttake();
                    robot.lights.onReverseOuttakeStart();
                },
                robot::endReverseOuttake));
        manager.scheduler().addTrigger(reverseOuttake);

        // Re-zero (dpad right/left, held): lights on press, pose snap
        // every tick while held, as before.
        Trigger reZeroGarden = new Trigger(() -> controls.secondary().dpad_right, manager.scheduler(), Clock.SYSTEM);
        reZeroGarden.whileTrue(Commands.startRun(robot.lights::onReZero, robot::reZeroGarden));
        manager.scheduler().addTrigger(reZeroGarden);

        Trigger reZeroLoading = new Trigger(() -> controls.secondary().dpad_left, manager.scheduler(), Clock.SYSTEM);
        reZeroLoading.whileTrue(Commands.startRun(robot.lights::onReZero, robot::reZeroCornerNextToLoadingZone));
        manager.scheduler().addTrigger(reZeroLoading);

        // Park (dpad up): schedule the retry command. Stop park (dpad
        // down): cancel it and the follower's park path.
        Trigger park = new Trigger(() -> controls.primary().dpad_up, manager.scheduler(), Clock.SYSTEM);
        park.onTrue(parkCommand);
        manager.scheduler().addTrigger(park);

        Trigger stopPark = new Trigger(() -> controls.primary().dpad_down, manager.scheduler(), Clock.SYSTEM);
        stopPark.onTrue(Commands.runOnce(() -> {
            manager.cancel(parkCommand);
            robot.cancelPark();
            parkingRequested = false;
            parkingInitiated = false;
            automaticParkingCancelled = true;
        }));
        manager.scheduler().addTrigger(stopPark);
    }

    /** One tick of driving: sticks, Y auto-align, shooter spin, auto-park request. */
    private void driveLoop() {
        controls.update();

        robot.setTeleOpDrive(
                new DriveVector(
                        -controls.primary().left_stick_y,
                        -controls.primary().left_stick_x,
                        -controls.primary().right_stick_x
                ),
                true, false
        );
        if (controls.autoAlignAndStartShoot().isPressed()) {
            robot.setAutoAlign();
        }
        if (controls.autoAlignAndStartShoot().risingEdge()) robot.lights.onShootStart();
        if (controls.keepShooterOn().risingEdge()) robot.lights.onShootStart();

        // Y (auto-align) also spins the shooter up while held, so Y
        // alone can shoot; A is the latching toggle for the same.
        robot.setSmartShooterControl(
                controls.keepShooterOn().isPressed()
                        || controls.autoAlignAndStartShoot().isPressed()
        );
        if (controls.keepShooterOn().fallingEdge()
                && !controls.autoAlignAndStartShoot().isPressed()) robot.shooter.setVelocity(0, 0);
        if (controls.autoAlignAndStartShoot().fallingEdge()
                && !controls.keepShooterOn().isPressed()) robot.shooter.setVelocity(0, 0);

        if (started && !automaticParkingCancelled && robot.shouldPark()
                && !parkingInitiated && !manager.scheduler().isScheduled(parkCommand)) {
            manager.schedule(parkCommand);
        }
    }

    /** One tick of transfer feeding: reverse blocks, B panics, triggers/Y feed. */
    private void feedLoop() {
        if (controls.reverseOuttake().isPressed()) {
            // Reversing the outtake to clear a jam: feed buttons are ignored.
        } else if (controls.panicShoot().isPressed()) {
            if (controls.panicShoot().risingEdge()) {
                robot.panic();
                robot.lights.onPanicStart();
            }
        } else {
            if (controls.runTransferFast().isPressed()
                    && robot.shooter.upToSpeed()
                    && robot.transfer.mode() != Transfer.Mode.REVERSE_EJECT) {
                robot.transfer.shootAll();
            }

            if (controls.runTransferSlow().isPressed()
                    && robot.transfer.mode() != Transfer.Mode.REVERSE_EJECT) {
                robot.transfer.slow();
            }

            if (controls.autoAlignAndStartShoot().isPressed()
                    && robot.shooter.upToSpeed()
                    && robot.transfer.mode() != Transfer.Mode.REVERSE_EJECT) {
                robot.transfer.shootAll();
            }

            if ((controls.runTransferFast().fallingEdge()
                    || controls.runTransferSlow().fallingEdge()
                    || controls.autoAlignAndStartShoot().fallingEdge())
                    && !controls.runTransferFast().isPressed()
                    && !controls.runTransferSlow().isPressed()
                    && !controls.autoAlignAndStartShoot().isPressed()
                    && robot.transfer.mode() != Transfer.Mode.REVERSE_EJECT) {
                robot.transfer.stop();
            }

            if (controls.panicShoot().fallingEdge()) robot.endPanicShoot();
        }
    }

    @Override
    protected void onLoop() {
        // ArchitecturalOpMode has no start() hook: robot.start() (match
        // timer, lights, vision) runs on the first loop tick, before the
        // first robot.update() that reads the timer.
        if (!started) {
            robot.start();
            started = true;
        }

        robot.telemetry.addData(
                "Parking",
                "requested=%s started=%s auto cancelled=%s",
                parkingRequested,
                parkingInitiated,
                automaticParkingCancelled
        );

        if (robot.opModeTimeRemaining() <= 0) {
            robot.stopEverything();
            OpModeStorage.endMatch();
            interrupt();
            return;
        }

        robot.update();
        robot.telemetry.update();
    }

    @TeleOp(name = CompTeleNames.ONE_DRIVER, group = Constants.COMPETITION_GROUP)
    public static class MainTeleOneController extends MainTeleOp {
        public MainTeleOneController() {
            this.oneController = true;
        }
    }
}
