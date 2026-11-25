package org.firstinspires.ftc.teamcode.mechanisms;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.localization.MotionState;
import com.pedropathing.math.Twist;
import com.pedropathing.math.Velocity;
import com.pedropathing.paths.Path;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.rmsftc.utils.ftc.DaemonManager;
import com.rmsftc.utils.ftc.daemons.FTCDaemon;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.planners.Constants;
import org.firstinspires.ftc.teamcode.pedro.Follower;
import com.rmsftc.utils.DriveVector;
import org.firstinspires.ftc.teamcode.util.SimEnvironment;
import org.firstinspires.ftc.teamcode.util.poses.PoseFactory;
import org.firstinspires.ftc.teamcode.planners.CommonKt;
import com.rmsftc.utils.AllianceColor;
import com.rmsftc.utils.Clock;
import com.rmsftc.utils.MatchMode;
import com.rmsftc.utils.RobotMode;
import org.firstinspires.ftc.teamcode.util.game.Field;
import com.rmsftc.utils.math.Pose;
import org.firstinspires.ftc.teamcode.util.shotcalc.AutoAimerCalculator;

import java.util.List;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.LinkedHashMap;
import java.util.Map;

public class Robot {
    /** The live Robot, for the static daemon entry points below. */
    private static Robot current;
    private static final List<String> ROBOT_DAEMON_ORDER = Arrays.asList(
            "robot-clock", "robot-battery", "robot-teleop-drive", "robot-vision",
            "robot-hive-state", "robot-aimer", "robot-follower", "robot-smart-control",
            "robot-mechanisms", "robot-lights", "robot-debug");
    private List<DaemonManager.RegisteredDaemon> orderedDaemons;
    private boolean running;

    public RobotMode robotMode;
    public Follower follower;
    public Intake intake;
    public Transfer transfer;
    public Shooter shooter;
    public Lights lights;
    public Vision vision;
    public AutoAimerCalculator autoAimerCalculator;
    public Field field;
    public HardwareMap.DeviceMapping<VoltageSensor> voltageSensors;
    public MultipleTelemetry telemetry;
    public List<LynxModule> allHubs;
    public AllianceColor allianceColor;
    public MatchMode matchMode = MatchMode.QUALIFICATIONS;

    public DriveVector drivePowers;
    public boolean holdIfNoInput;
    public boolean setTeleOpDrive;
    private boolean autoAlignRequested = false;

    /** FTCUtils clock: Helper tracks per-loop dt; startNanos anchors
     * op-mode elapsed time. Both (re)started in {@link #start()}. */
    public final Clock clock = Clock.SYSTEM;
    private Clock.Helper clockHelper;
    private long startNanos;
    // 0, not 1: a first-loop dt of 1 s would corrupt the first
    // setAutoAlign hold (calculateHold) and any first-tick control math.
    double dt = 0;

    public boolean smartIntakeControl = false;
    public boolean smartShooterControl = false;
    public boolean debug = false;

    public Robot(Telemetry telemetry, HardwareMap hardwareMap, AllianceColor color, RobotMode robotMode, Pose startingPose) {
        CommonKt.init(color);
        org.firstinspires.ftc.teamcode.pedro.Constants.PINPOINT_CONFIG.resetMode.set(robotMode==RobotMode.AUTONOMOUS? PinpointLocalizer.ResetMode.RESET_AND_RECALIBRATE_IMU: PinpointLocalizer.ResetMode.NONE);
        org.firstinspires.ftc.teamcode.pedro.Constants.FOLLOWER_BRAKE_AT_END.set(true);
        follower = org.firstinspires.ftc.teamcode.pedro.Constants.createFollower(hardwareMap);
        follower.holdEnd.set(false);
        follower.localizer.setPose(startingPose);

        this.allianceColor = color;
        autoAimerCalculator = new AutoAimerCalculator(allianceColor);
        autoAimerCalculator.initializeCorrectedPose(startingPose);
        autoAimerCalculator.useFollowerHold = true; // IMPORTANT

        intake = new Intake(hardwareMap, color);
        transfer = new Transfer(hardwareMap);
        shooter = new Shooter(hardwareMap, color);
        // ftcsim provides no Limelight or Prism device, so in sim both are
        // replaced with no-hardware stubs; on the robot this is unchanged.
        lights = SimEnvironment.isSimulation()
                ? Lights.stub(hardwareMap, color) : new Lights(hardwareMap, color);
        vision = SimEnvironment.isSimulation()
                ? Vision.stub(hardwareMap, color) : new Vision(hardwareMap, color);
        field = new Field(color, 17, 17);
        this.robotMode = robotMode;
        voltageSensors = hardwareMap.voltageSensor;

        drivePowers = DriveVector.ZERO;
        holdIfNoInput = false;
        setTeleOpDrive = false;

        // Bulk caching
        allHubs = hardwareMap.getAll(LynxModule.class);
        allHubs.forEach(lynxModule -> lynxModule.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO));


        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        this.telemetry.clearAll();
        current = this;
    }

    public void start() {
        // Filter to the 11 robot daemons only: the global registry also
        // holds library daemons (FTCUtils ClockManager, BA architecture-tick)
        // that must not run inside the robot loop. A duplicate registration
        // (e.g. a re-scan) is deduped instead of throwing, so start() does
        // not die on loop 1; a missing robot daemon still throws.
        Map<String, DaemonManager.RegisteredDaemon> byName = new LinkedHashMap<>();
        for (DaemonManager.RegisteredDaemon daemon : DaemonManager.unmodifiable()) {
            byName.putIfAbsent(daemon.name, daemon);
        }
        orderedDaemons = new ArrayList<>();
        for (String name : ROBOT_DAEMON_ORDER) {
            DaemonManager.RegisteredDaemon daemon = byName.get(name);
            if (daemon == null) {
                throw new IllegalStateException("Expected one " + name + " daemon; found 0");
            }
            orderedDaemons.add(daemon);
        }
        orderedDaemons.sort(Comparator.comparingInt(d -> daemonPriority(d.name)));
        clockHelper = new Clock.Helper(clock);
        startNanos = clock.nowNanos();
        lights.start();
        vision.start();
        running = true;
    }

    /**
     * One loop of robot work: ticks the FTCUtils daemon registry once.
     * All periodic work runs in the per-step daemons below (plus
     * ClockManager's time snapshot, which nothing here depends on).
     */
    public void update() {
        if (!running) {
            throw new IllegalStateException("Robot must be started before update()");
        }
        orderedDaemons.forEach(d -> d.daemon.execute());
    }

    static int daemonPriority(String name) {
        return ROBOT_DAEMON_ORDER.indexOf(name);
    }


    /** Step 1: advance the loop clock and cache dt for this loop. */
    @FTCDaemon(name = "robot-clock")
    public static void clockDaemon() {
        Robot robot = current;
        if (robot != null && robot.running) robot.stepClock();
    }

    private void stepClock() {
        clockHelper.update();
        dt = clockHelper.dtSeconds();
    }

    /** Step 2: refresh the cached battery voltage (300 ms cache). */
    @FTCDaemon(name = "robot-battery")
    public static void batteryDaemon() {
        Robot robot = current;
        if (robot != null && robot.running) robot.stepBattery();
    }

    private void stepBattery() {
        getBatteryVoltage();
    }

    /** Step 3: apply this loop's teleop drive input, if any. */
    @FTCDaemon(name = "robot-teleop-drive")
    public static void teleOpDriveDaemon() {
        Robot robot = current;
        if (robot != null && robot.running) robot.stepTeleOpDrive();
    }

    private void stepTeleOpDrive() {
        if (setTeleOpDrive) {
            if (holdIfNoInput) {
                follower.driveOrHold(drivePowers);
            } else {
                follower.manual(drivePowers);
            }
            setTeleOpDrive = false;
        }
    }

    /** Step 4: update vision with the current heading. */
    @FTCDaemon(name = "robot-vision")
    public static void visionDaemon() {
        Robot robot = current;
        if (robot != null && robot.running) robot.stepVision();
    }

    private void stepVision() {
        vision.update(getHeading());
    }

    /** Step 5: refresh the hive tip state from the closest hive pose. */
    @FTCDaemon(name = "robot-hive-state")
    public static void hiveStateDaemon() {
        Robot robot = current;
        if (robot != null && robot.running) robot.stepHiveState();
    }

    private void stepHiveState() {
        updateHiveStateFromClosestHivePose();
    }

    /** Step 6: feed the hive state and pose to the auto-aimer. */
    @FTCDaemon(name = "robot-aimer")
    public static void aimerDaemon() {
        Robot robot = current;
        if (robot != null && robot.running) robot.stepAimer();
    }

    private void stepAimer() {
        autoAimerCalculator.setHiveState(field.getHiveState());
        autoAimerCalculator.updatePedro(getPose(), getVelocity(), dt);
    }

    /** Step 7: run the follower (path following / teleop drive). */
    @FTCDaemon(name = "robot-follower")
    public static void followerDaemon() {
        Robot robot = current;
        if (robot != null && robot.running) robot.stepFollower();
    }

    private void stepFollower() {
        // Y released (setAutoAlign not called this loop): drop the
        // shoot-on-the-move heading override before the follower runs.
        if (!autoAlignRequested) follower.headingOverride = null;
        follower.update(dt);
        autoAlignRequested = false;
    }

    /**
     * Step 8: automatic intake/shooter control. Manual overrides
     * (panic / reverse outtake) still win inside smartShooterUpdate.
     */
    @FTCDaemon(name = "robot-smart-control")
    public static void smartControlDaemon() {
        Robot robot = current;
        if (robot != null && robot.running) robot.stepSmartControl();
    }

    private void stepSmartControl() {
        if (smartIntakeControl) smartIntakeUpdate();
        if (smartShooterControl) smartShooterUpdate();
    }

    /** Step 9: per-loop mechanism updates. */
    @FTCDaemon(name = "robot-mechanisms")
    public static void mechanismsDaemon() {
        Robot robot = current;
        if (robot != null && robot.running) robot.stepMechanisms();
    }

    private void stepMechanisms() {
        intake.update(/*follower.twist()*/);
        transfer.update();
        shooter.update();
    }

    /** Step 10: lights state machine + LED output. */
    @FTCDaemon(name = "robot-lights")
    public static void lightsDaemon() {
        Robot robot = current;
        if (robot != null && robot.running) robot.stepLights();
    }

    private void stepLights() {
        lightsControl();
        lights.update();
    }

    /** Step 11: debug telemetry, when enabled. */
    @FTCDaemon(name = "robot-debug")
    public static void debugDaemon() {
        Robot robot = current;
        if (robot != null && robot.running) robot.stepDebug();
    }

    private void stepDebug() {
        if (debug) debug();
    }


    public void debug() {
        telemetry.addData("=== UTILS DEBUG ===", "\n Current Battery Voltage: "+ getBatteryVoltage()+"\n Dt: "+dt*1000+"\n OpMode time (sec): "+ getOpModeTimeSeconds()+ "\n Time Remaining (sec): " + opModeTimeRemaining() + "\n Robot Mode: " +robotMode + "\n Alliance Color: " + allianceColor + "\n TeleOp brake: "+org.firstinspires.ftc.teamcode.pedro.Constants.FOLLOWER_MANUAL_BRAKE_MODE.get());telemetry.addData("\n === DRIVE POWERS ===", drivePowers.toString());
        telemetry.addData("=== FOLLOWER DEBUG ===", follower.debug().toString());
        telemetry.addData("Pose", getPose().toString());
        telemetry.addData("Hive state (closest hive pose)", field.getHiveState()
                + " (untipped " + String.format("%.1f", getPose().distance(Field.HiveState.UNTIPPED.goalPose(allianceColor)))
                + " in, tipped " + String.format("%.1f", getPose().distance(Field.HiveState.TIPPED.goalPose(allianceColor))) + " in)");
        telemetry.addData("=== INTAKE DEBUG ===", "\n"+intake.getDebugString());
        telemetry.addData("=== TRANSFER DEBUG ===", "\n"+transfer.getDebugString());
        telemetry.addData("=== SHOOTER DEBUG ===", "\n"+shooter.getDebugString());
        telemetry.addData("=== LIGHTS DEBUG ===", "\n"+lights.getDebugString());
        telemetry.addData("=== VISION DEBUG ===", "\n"+vision.getDebugString());
        telemetry.addData("=== AUTO AIMER CALCULATOR DEBUG ===", "\n"+autoAimerCalculator.getDebugString());
        telemetry.addData("=== SHOT CALC DEBUG ===", "\n"+shooter.shotCalculator.getDebugString());
    }
    public void stopAllSubsystems() {
        //allHubs.forEach(LynxModule::close);
        intake.stopImmediately();
        transfer.stopImmediately();
        shooter.stopImmediately();
        lights.stop();
        vision.stop();
    }

    public void stopEverything() {
        running = false;
        smartIntakeControl = false;
        smartShooterControl = false;
        setTeleOpDrive = false;
        if (current == this) current = null;
        stopAllSubsystems();
        follower.stop();
        follower.drivetrain.drive(DriveVector.ZERO, false); // this coasts
    }

    /* UTILS/MATH/GEOM */

    double lastVolt = Double.NaN;
    boolean isBatteryLowPower = false;
    double lastBatteryTime = clock.nowSeconds() * 1000.0;
    public double getBatteryVoltage() {
        double cur = clock.nowSeconds() * 1000.0;
        if (cur - lastBatteryTime < Constants.Robot.Battery.CACHE_MS && !Double.isNaN(lastVolt)) {
            return lastVolt;
        }
        lastBatteryTime=cur;
        double result = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : voltageSensors) {
            double voltage = sensor.getVoltage();
            if (voltage > 0) {
                result = Math.min(result, voltage);
            }
        }
        if (result <= Constants.Robot.Battery.LOW_POWER_VOLTAGE_THRESHOLD-Constants.Robot.Battery.LOW_POWER_VOLTAGE_TOLERANCE) isBatteryLowPower=true;
        else if (result >= Constants.Robot.Battery.LOW_POWER_VOLTAGE_THRESHOLD+Constants.Robot.Battery.LOW_POWER_VOLTAGE_TOLERANCE) isBatteryLowPower=false;
        lastVolt = result;
        return result;
    }

    public double getOpModeTimeSeconds() {
        return (clock.nowNanos() - startNanos) / 1e9;
    }

    public double opModeTimeRemaining() {
        return robotMode.getTimeLimit()-getOpModeTimeSeconds();
    }

    public double getDt() {
        return dt;
    }

    public double distanceToLoadingZone() {
        return field.getClosestLoadingZoneDistanceToRobot(getPoseCompensatedForVelocity());
    }

    /* FOLLOWER */

    public void setManualBrake(boolean manualBrake) {
        org.firstinspires.ftc.teamcode.pedro.Constants.FOLLOWER_MANUAL_BRAKE_MODE.set(manualBrake);
    }

    public void setAutoMaxPathSpeed(double speed) {
        org.firstinspires.ftc.teamcode.pedro.Constants.FOLLOWER_MAX_PATH_SPEED.set(speed);
    }

    public Pose getPose() {
        return PoseFactory.of(follower.localizer.pose());
    }

    public double getHeading() {
        return getPose().heading();
    }

    public double getHeading(AngleUnit unit) {
        return unit.fromRadians(getHeading());
    }

    public MotionState getMotionState() { return follower.localizer.state(); }

    /** Returns the Robot's velocity, relative to the field */
    public Velocity getVelocity() {
        return follower.localizer.velocity();
    }

    /** Returns the Robot's velocity, relative to the robot */
    public Twist getTwist() {
        return follower.localizer.twist();
    }

    public Velocity getDeltaPose() {
        return getVelocity().scale(dt);
    }

    public Pose getPoseCompensatedForVelocity() {
        //return PoseFactory.of(follower.localizer.pose().exp(follower.velocity(), dt));
        return autoAimerCalculator.getCorrectedPoseOrFallback(follower.localizer.pose());
    }

    public void breakFollowing() {follower.stop();}
    public void setTeleOpDrive(DriveVector DriveVector, boolean fieldCentric, boolean holdIfNoInput) {
        if (follower.following()) return;
        if (follower.holding() && follower.holdEnd.get() && !follower.hasInput(DriveVector, Constants.Robot.Drive.INPUT_DEADBAND)) return;
        this.drivePowers = DriveVector;
        if (fieldCentric) {
            this.drivePowers.fieldCentric(getHeading(), allianceColor);
        }
        this.drivePowers.scale(!isBatteryLowPower ? Constants.Robot.Drive.SCALE : Constants.Robot.Drive.SCALE);
        this.holdIfNoInput = holdIfNoInput;
        this.setTeleOpDrive = true;
    }

    public void setAutoAlign() {
        autoAlignRequested = true;
        if (follower.following()) {
            // Shoot-on-the-move: keep following the path (calculatePath
            // still drives translation), but face the goal instead of the
            // path's heading interpolator. Must NOT set setTeleOpDrive --
            // that switches to manual() and cancels the follow (the old
            // Y-cancels-park bug).
            follower.headingOverride = autoAimerCalculator.getTargetHeading();
            return;
        }
        if (autoAimerCalculator.useFollowerHold) {
            drivePowers.turn(follower.algorithm.calculateHold(
                    follower.drivetrain,
                    getPoseCompensatedForVelocity().withHeading(autoAimerCalculator.getTargetHeading()),
                    follower.localizer.state(),
                    false, // Use hold scaling
                    getDt() // in seconds
            ).turn()*Constants.Robot.Drive.AUTO_ALIGN_DAMPENING); // a dampening factor
        } else {
            drivePowers.turn(autoAimerCalculator.calculateTurningPower(getHeading()));
        }
        this.setTeleOpDrive = true;
    }

    public boolean reZeroCornerNextToLoadingZone() {
//        if (!Constants.UtilityPoses.isCornerLoadingAreaConfigured()) {
//            telemetry.addLine("Corner loading re-zero rejected: configure Constants.UtilityPoses.cornerLoadingArea.");
//            return false;
//        }
        follower.localizer.setPose(Constants.UtilityPoses.cornerLoadingArea);
        autoAimerCalculator.initializeCorrectedPose(Constants.UtilityPoses.cornerLoadingArea);
        telemetry.addData("Corner loading re-zero applied", Constants.UtilityPoses.cornerLoadingArea);
        return true;
    }

    public boolean reZeroGarden() {
//        if (!Constants.UtilityPoses.isCornerGardenAreaConfigured()) {
//            telemetry.addLine("Garden corner re-zero rejected: configure Constants.UtilityPoses.cornerGardenArea.");
//            return false;
//        }
        follower.localizer.setPose(Constants.UtilityPoses.cornerGardenArea);
        autoAimerCalculator.initializeCorrectedPose(Constants.UtilityPoses.cornerGardenArea);
        telemetry.addData("Garden corner re-zero applied", Constants.UtilityPoses.cornerGardenArea);
        return true;
    }

    /**
     * Drives to the loading zone (park), or holds if already overlapping it.
     * Never throws: any path-generation failure returns false, and every
     * caller retries on a 0.5 s interval. A failure before a new follow
     * starts (overlap check, path generation, no route) leaves a running
     * path alone; only a failure from {@code follower.follow()} itself
     * stops the follower, so a half-started path cannot run.
     */
    public boolean moveToParkZone() {
        boolean previousHoldEnd = follower.holdEnd.get();
        boolean previousBrakeAtEnd = org.firstinspires.ftc.teamcode.pedro.Constants.FOLLOWER_BRAKE_AT_END.get();
        follower.holdEnd.set(true); // alliance partner does not push
        org.firstinspires.ftc.teamcode.pedro.Constants.FOLLOWER_BRAKE_AT_END.set(false);
        try {
            Pose correctedPose = getPoseCompensatedForVelocity();
            if (field.isRobotOverlappingLoadingZone(correctedPose)) {
                follower.hold(getPose());
                setTeleOpDrive = false;
                return true;
            }
            // Generate the path before starting any follow: a failure here
            // means no new path was started.
            Path path = field.generatePathToLoadingZoneShortest(correctedPose, 3); // TODO TUNE INNER VAL
            try {
                follower.follow(path);
            } catch (RuntimeException followFailure) {
                // A follow was attempted: stop so a half-started path cannot run.
                follower.stop();
                throw followFailure;
            }
            // Prevent the queued manual command from replacing the path this loop.
            setTeleOpDrive = false;
            return true;
        } catch (RuntimeException failure) {
            // Path generation (or the overlap check) failed before any new
            // follow started: restore the shared follower flags and report
            // park as unavailable. Never throws.
            follower.holdEnd.set(previousHoldEnd);
            org.firstinspires.ftc.teamcode.pedro.Constants.FOLLOWER_BRAKE_AT_END.set(previousBrakeAtEnd);
            telemetry.addData("Park path unavailable", failure.getMessage());
            return false;
        }
    }

    public void cancelPark() {
        follower.stop();
        follower.drivetrain.stop();
        follower.holdEnd.set(false);
        org.firstinspires.ftc.teamcode.pedro.Constants.FOLLOWER_BRAKE_AT_END.set(true);
    }

    public boolean inParkZone() {
        return field.isRobotOverlappingLoadingZone(getPoseCompensatedForVelocity());
    }

    public boolean shouldPark() {
        return shouldAutomaticallyPark(matchMode, opModeTimeRemaining());
    }

    static boolean shouldAutomaticallyPark(MatchMode matchMode, double timeRemaining) {
        return matchMode == MatchMode.QUALIFICATIONS
                && timeRemaining > 0.0
                && timeRemaining <= Constants.Robot.AutoPark.START_TIME_REMAINING_SEC;
    }

    /** Hive state from whichever hive pose the robot is closest to: the
     * untipped hive goal means the hive is UNTIPPED, the tipped hive goal
     * means TIPPED. The hive goals are the target points on the hive itself
     * (Field.HiveState.goalPose, alliance-correct), so this follows the face
     * the robot is actually nearest. This is the sole source of the hive
     * state — vision never writes it, and there is no manual flip to be
     * undone. Ties go to the untipped goal (UNTIPPED), matching the field's
     * initial state. */
    private void updateHiveStateFromClosestHivePose() {
        Pose pose = getPose();
        double untippedDistance = pose.distance(Field.HiveState.UNTIPPED.goalPose(allianceColor));
        double tippedDistance = pose.distance(Field.HiveState.TIPPED.goalPose(allianceColor));
        field.setHiveState(untippedDistance <= tippedDistance
                ? Field.HiveState.UNTIPPED
                : Field.HiveState.TIPPED);
    }

    /* INTAKE */
    public void intakeEject() {
        intake.forceEject();
        transfer.reverse();
    }
    /** End an eject: the intake goes to the given mode (taken from the
     * intake toggle button in teleop) instead of the mode saved when the
     * eject started, so toggling the intake off mid-eject actually stops
     * it instead of restoring full power. */
    public void restoreIntake(Intake.Mode mode) {
        intake.stopEjecting(mode);
        transfer.stop();
    }

    // smart intake control basically has intake on the whole time except while ejecting and battery voltage low
    public void setSmartIntakeControl(boolean set) {
        if (set) intake.setFullPower();
        smartIntakeControl = set;
    }

    public void smartIntakeUpdate() {
        if (!smartIntakeControl) return;
        if (!intake.isEjecting()) {
            if (!isBatteryLowPower) {
                if (intake.mode() != Intake.Mode.FULL_POWER) intake.setFullPower();
            } else {
                if (intake.mode() != Intake.Mode.LOW_POWER) intake.setLowPower();
            }
        }
    }

    /* TRANSFER */

    public void panic() {
        shooter.panic();
        transfer.panicShoot();
    }
    public void endPanicShoot() {
        shooter.panicking = false;
        shooter.setVelocity(0, 0);
        transfer.stop();
    }
    /** Reverse both outtake motors to clear a jammed ball; stop the transfer so nothing feeds into the jam. */
    public void reverseOuttake() {
        shooter.reverseOuttake();
        transfer.stop();
    }
    public void endReverseOuttake() {
        shooter.reversing = false;
        shooter.setVelocity(0, 0);
        transfer.stop();
    }

    /* SHOOTER */

    public void setSmartShooterControl(boolean set) {
        smartShooterControl = set;
    }

    public void smartShooterUpdate() {
        if (shooter.panicking || shooter.reversing) return;
        shooter.solveAndSetVelocity(
                getPose(),
                getVelocity(),
                field.getHiveState(),
                getDt()
        );
    }

    /* LIGHTS */

    private void lightsControl() {
        if (opModeTimeRemaining() <= Constants.Robot.Lights.FLASH_RED_SECONDS_REMAINING) {lights.flashRed();}
        else if (opModeTimeRemaining() >= Constants.Robot.Lights.ONE_MINUTE_WINDOW_START_SEC && opModeTimeRemaining() <= Constants.Robot.Lights.ONE_MINUTE_WINDOW_END_SEC) {
            lights.oneMinThing();
        } else if (shooter.upToSpeed()) {
            lights.displayGreen();
        } else {
            lights.off();
        }
    }

    /* VISION
     * Vision runs every loop (Limelight pipeline, tag detection, hive lineup)
     * but it never moves the robot's pose: there is no vision relocalization,
     * automatic or manual. The pose comes only from the Pinpoint localizer
     * (plus the driver's manual corner re-zeros, which set fixed poses, not
     * vision estimates). The hive state likewise comes from the robot's pose
     * (updateHiveStateFromClosestHivePose), not from vision. */
}
