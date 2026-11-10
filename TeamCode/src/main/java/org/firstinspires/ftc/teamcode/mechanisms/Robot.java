package org.firstinspires.ftc.teamcode.mechanisms;


import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.localization.MotionState;
import com.pedropathing.math.Twist;
import com.pedropathing.math.Velocity;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.pedro.Follower;
import org.firstinspires.ftc.teamcode.util.hardware.gamepad.DriveVector;
import org.firstinspires.ftc.teamcode.util.localization.VisionLocalizationMath;
import org.firstinspires.ftc.teamcode.util.poses.PoseFactory;
import org.firstinspires.ftc.teamcode.planners.CommonKt;
import org.firstinspires.ftc.teamcode.planners.UtilityPoses;
import org.firstinspires.ftc.teamcode.util.AllianceColor;
import org.firstinspires.ftc.teamcode.util.MatchMode;
import org.firstinspires.ftc.teamcode.util.RobotMode;
import org.firstinspires.ftc.teamcode.util.Timer;
import org.firstinspires.ftc.teamcode.util.game.Field;
import org.firstinspires.ftc.teamcode.util.poses.Pose2d;
import org.firstinspires.ftc.teamcode.util.shotcalc.AutoAimerCalculator;
import org.firstinspires.ftc.teamcode.util.vars.Variables;

import java.util.List;
import java.util.concurrent.TimeUnit;

public class Robot {
    public static boolean AUTO_RELOCALIZE_WITH_VISION = false;
    public static double MAX_AUTO_RELOCALIZE_POSITION_DELTA_IN = 24.0;
    public static double MAX_AUTO_RELOCALIZE_HEADING_DELTA_DEG = 40.0;

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
    public static final double LOW_POWER_VOLTAGE_THRESHOLD = 11.5; // when it starts to become low power
    public static final double LOW_POWER_VOLTAGE_TOLERANCE = 0.25;
    public List<LynxModule> allHubs;
    public AllianceColor allianceColor;
    public MatchMode matchMode = MatchMode.QUALIFICATIONS;

    public DriveVector drivePowers;
    public boolean holdIfNoInput;
    public boolean setTeleOpDrive;

    public Timer opModeTimer;
    double dt = 1;

    public boolean smartIntakeControl = false;
    public boolean smartShooterControl = false;
    public boolean debug = false;

    public Robot(Telemetry telemetry, HardwareMap hardwareMap, AllianceColor color, RobotMode robotMode, Pose2d startingPose) {
        CommonKt.init(color);
        Constants.PINPOINT_CONFIG.resetMode.set(robotMode==RobotMode.AUTONOMOUS? PinpointLocalizer.ResetMode.RESET_AND_RECALIBRATE_IMU: PinpointLocalizer.ResetMode.NONE);
        Constants.FOLLOWER_BRAKE_AT_END.set(true);
        follower = Constants.createFollower(hardwareMap);
        follower.holdEnd.set(false);
        follower.localizer.setPose(startingPose);

        this.allianceColor = color;
        autoAimerCalculator = new AutoAimerCalculator(allianceColor);
        autoAimerCalculator.initializeCorrectedPose(startingPose);
        autoAimerCalculator.useFollowerHold = true; // IMPORTANT

        intake = new Intake(hardwareMap, color);
        transfer = new Transfer(hardwareMap);
        shooter = new Shooter(hardwareMap, color);
        lights = new Lights(hardwareMap, color);
        vision = new StubVision(hardwareMap, color);
        field = new Field(color, 13.835, 17.008);
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
//        Variables.registerAllToFtcDash();
    }

    public void start() {
        opModeTimer = new Timer(TimeUnit.SECONDS);
        opModeTimer.reset();
        lights.start();
        vision.start();
    }

    public void update() {
        dt = opModeTimer.dt(TimeUnit.SECONDS);
        getBatteryVoltage();
        if (setTeleOpDrive) {
            if (holdIfNoInput) {
                follower.driveOrHold(drivePowers);
            } else {
                follower.manual(drivePowers);
            }
            setTeleOpDrive = false;
        }

        vision.update(getHeading());
        vision.applyInferredHiveState(field);
        if (AUTO_RELOCALIZE_WITH_VISION && vision.hasReliableLocalization()) {
            applyVisionLocalizationIfWithinBounds();
        }

        autoAimerCalculator.setHiveState(field.getHiveState());
        autoAimerCalculator.updatePedro(getPose(), getVelocity(), dt);

        follower.update(dt);
        if (smartIntakeControl) smartIntakeUpdate();
        if (smartShooterControl) smartShooterUpdate();
        intake.update(/*follower.twist()*/);
        transfer.update();
        shooter.update();
        lightsControl();
        lights.update();
        if (debug) debug();
    }


    public void debug() {
        telemetry.addData("=== UTILS DEBUG ===", "\n Current Battery Voltage: "+ getBatteryVoltage()+"\n Dt: "+dt*1000+"\n OpMode time (sec): "+ opModeTimer.get()+ "\n Time Remaining (sec): " + opModeTimeRemaining() + "\n Robot Mode: " +robotMode + "\n Alliance Color: " + allianceColor + "\n TeleOp brake: "+Constants.FOLLOWER_MANUAL_BRAKE_MODE.get());telemetry.addData("\n === DRIVE POWERS ===", drivePowers.toString());
        telemetry.addData("=== FOLLOWER DEBUG ===", follower.debug().toString());
        telemetry.addData("Pose", getPose().toString());
        telemetry.addData("=== INTAKE DEBUG ===", "\n"+intake.getDebugString());
        telemetry.addData("=== TRANSFER DEBUG ===", "\n"+transfer.getDebugString());
        telemetry.addData("=== SHOOTER DEBUG ===", "\n"+shooter.getDebugString());
        telemetry.addData("=== LIGHTS DEBUG ===", "\n"+lights.getDebugString());
        telemetry.addData("=== VISION DEBUG ===", "\n"+vision.getDebugString());
        telemetry.addData("=== AUTO AIMER CALCULATOR DEBUG ===", "\n"+autoAimerCalculator.getDebugString());
        telemetry.addData("=== SHOT CALC DEBUG", shooter.shotCalculator.getDebugString());
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
        stopAllSubsystems();
        follower.stop();
        follower.drivetrain.drive(DriveVector.ZERO, false); // this coasts
    }

    /* UTILS/MATH/GEOM */

    double lastVolt = Double.NaN;
    boolean isBatteryLowPower = false;
    double lastBatteryTime = System.currentTimeMillis();
    public double getBatteryVoltage() {
        double cur=System.currentTimeMillis();
        if (cur - lastBatteryTime < 300 && !Double.isNaN(lastVolt)) {
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
        if (result <= LOW_POWER_VOLTAGE_THRESHOLD-LOW_POWER_VOLTAGE_TOLERANCE) isBatteryLowPower=true;
        else if (result >= LOW_POWER_VOLTAGE_THRESHOLD+LOW_POWER_VOLTAGE_TOLERANCE) isBatteryLowPower=false;
        lastVolt = result;
        return result;
    }

    public double getOpModeTimeSeconds() {
        return opModeTimer.get(TimeUnit.SECONDS);
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
        Constants.FOLLOWER_MANUAL_BRAKE_MODE.set(manualBrake);
    }

    public void setAutoMaxPathSpeed(double speed) {
        Constants.FOLLOWER_MAX_PATH_SPEED.set(speed);
    }

    public Pose2d getPose() {
        return PoseFactory.of(follower.localizer.pose());
    }

    public double getHeading() {
        return getPose().getHeading();
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

    public Pose2d getPoseCompensatedForVelocity() {
        //return PoseFactory.of(follower.localizer.pose().exp(follower.velocity(), dt));
        return autoAimerCalculator.getCorrectedPoseOrFallback(follower.localizer.pose());
    }

    public void breakFollowing() {follower.stop();}
    public void setTeleOpDrive(DriveVector DriveVector, boolean fieldCentric, boolean holdIfNoInput) {
        if (follower.following()) return;
        this.drivePowers = DriveVector;
        if (fieldCentric) {
            this.drivePowers.fieldCentric(getHeading(), allianceColor);
        }
        this.drivePowers.scale(!isBatteryLowPower ? 0.8 : 0.4);
        this.holdIfNoInput = holdIfNoInput;
        this.setTeleOpDrive = true;
    }

    /** TODO Complete*/
    public void setAutoAlign() {
        if (autoAimerCalculator.useFollowerHold) {
            drivePowers.setTurn(follower.algorithm.calculateHold(
                    follower.drivetrain,
                    getPoseCompensatedForVelocity().withHeading(autoAimerCalculator.getTargetHeading()),
                    follower.localizer.state(),
                    false, // Use hold scaling
                    getDt() // in seconds
            ).turn());
        } else {
            drivePowers.setTurn(autoAimerCalculator.calculateTurningPower(getHeading()));
        }
    }

    public boolean reZeroCornerNextToLoadingZone() {
        if (!UtilityPoses.isCornerLoadingAreaConfigured()) {
            telemetry.addLine("Corner loading re-zero rejected: configure UtilityPoses.cornerLoadingArea.");
            return false;
        }
        follower.localizer.setPose(UtilityPoses.cornerLoadingArea);
        autoAimerCalculator.initializeCorrectedPose(UtilityPoses.cornerLoadingArea);
        telemetry.addData("Corner loading re-zero applied", UtilityPoses.cornerLoadingArea);
        return true;
    }

    public boolean reZeroGarden() {
        if (!UtilityPoses.isCornerGardenAreaConfigured()) {
            telemetry.addLine("Garden corner re-zero rejected: configure UtilityPoses.cornerGardenArea.");
            return false;
        }
        follower.localizer.setPose(UtilityPoses.cornerGardenArea);
        autoAimerCalculator.initializeCorrectedPose(UtilityPoses.cornerGardenArea);
        telemetry.addData("Garden corner re-zero applied", UtilityPoses.cornerGardenArea);
        return true;
    }

    public boolean moveToParkZone() {
        Pose2d correctedPose = getPoseCompensatedForVelocity();
        if (field.isRobotOverlappingLoadingZone(correctedPose)) return true;
        boolean previousHoldEnd = follower.holdEnd.get();
        boolean previousBrakeAtEnd = Constants.FOLLOWER_BRAKE_AT_END.get();
        follower.holdEnd.set(true); // alliance partner does not push
        Constants.FOLLOWER_BRAKE_AT_END.set(false);
        try {
            follower.follow(field.generatePathToLoadingZoneShortest(correctedPose, 3));// TODO TUNE INNER VAL
            // Prevent the queued manual command from replacing the path this loop.
            setTeleOpDrive = false;
            return true;
        } catch (Field.NoRouteException exception) {
            follower.stop();
            follower.holdEnd.set(previousHoldEnd);
            Constants.FOLLOWER_BRAKE_AT_END.set(previousBrakeAtEnd);
            telemetry.addData("Park path unavailable", exception.getMessage());
            return false;
        }
    }

    public void cancelPark() {
        follower.stop();
        follower.holdEnd.set(false);
        Constants.FOLLOWER_BRAKE_AT_END.set(true);
    }

    public boolean inParkZone() {
        return field.isRobotOverlappingLoadingZone(getPoseCompensatedForVelocity());
    }
    public boolean shouldPark() {
        double timeToGetToPark = 2;

        // TODO Lets assume for now

        return Math.abs(timeToGetToPark-opModeTimeRemaining())<=1 && matchMode==MatchMode.QUALIFICATIONS;
    }

    public void toggleHiveState() {
        Field.HiveState nextHiveState = field.getHiveState() == Field.HiveState.TIPPED
                ? Field.HiveState.UNTIPPED
                : Field.HiveState.TIPPED;
        field.setHiveState(nextHiveState);
        telemetry.addData("Hive state", nextHiveState);
    }

    /* INTAKE */
    public void intakeEject() {
        intake.forceEject();
        transfer.reverse();
    }
    public void restoreIntake() {
        intake.stopEjecting();
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
        shooter.setVelocity(0);
        transfer.stop();
    }

    /* SHOOTER */

    public void setSmartShooterControl(boolean set) {
        smartShooterControl = set;
    }

    public void smartShooterUpdate() {
        if (shooter.panicking) return;
        shooter.solveAndSetVelocity(
                getPose(),
                getVelocity(),
                field.getHiveState(),
                getDt()
        );
    }

    /* LIGHTS */

    private void lightsControl() {
        if (opModeTimeRemaining() <= 7) {lights.flashRed();}
        else if (opModeTimeRemaining() >= 57 && opModeTimeRemaining() <= 63) {
            lights.oneMinThing();
        } else if (shooter.upToSpeed()) {
            lights.displayGreen();
        } else {
            lights.off();
        }
    }

    /* VISION */
    public boolean reZeroVision() {
        if (!vision.hasReliableLocalization()) {
            telemetry.addData("Vision re-zero rejected", vision.getLocalizationReliabilityReason());
            return false;
        }
        Pose2d visionPose = vision.getFieldPoseEstimate();
        follower.localizer.setPose(visionPose);
        autoAimerCalculator.initializeCorrectedPose(visionPose);
        telemetry.addData("Vision re-zero applied", visionPose);
        return true;
    }

    private void applyVisionLocalizationIfWithinBounds() {
        Pose2d currentPose = getPose();
        Pose2d visionPose = vision.getFieldPoseEstimate();

        double positionDelta = currentPose.distance(visionPose);
        double headingDeltaDeg = Math.abs(AngleUnit.DEGREES.fromRadians(
                VisionLocalizationMath.normalizeRadians(currentPose.heading() - visionPose.heading())
        ));

        if (positionDelta > MAX_AUTO_RELOCALIZE_POSITION_DELTA_IN) {
            telemetry.addData(
                    "Vision auto relocalize skipped",
                    "Position delta %.2f in exceeds %.2f in",
                    positionDelta,
                    MAX_AUTO_RELOCALIZE_POSITION_DELTA_IN
            );
            return;
        }
        if (headingDeltaDeg > MAX_AUTO_RELOCALIZE_HEADING_DELTA_DEG) {
            telemetry.addData(
                    "Vision auto relocalize skipped",
                    "Heading delta %.2f deg exceeds %.2f deg",
                    headingDeltaDeg,
                    MAX_AUTO_RELOCALIZE_HEADING_DELTA_DEG
            );
            return;
        }
        follower.localizer.setPose(visionPose);
        autoAimerCalculator.initializeCorrectedPose(visionPose);
    }
}
