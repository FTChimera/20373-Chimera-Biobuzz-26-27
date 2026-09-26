package org.firstinspires.ftc.teamcode.mechanisms;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierPoint;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.library.DriveVector;
import org.firstinspires.ftc.teamcode.library.ShotSolver;
import org.firstinspires.ftc.teamcode.library.Tuple;
import org.firstinspires.ftc.teamcode.library.game.RobotMode;
import org.firstinspires.ftc.teamcode.library.game.State;
import org.firstinspires.ftc.teamcode.library.robot.drivetrain.MecanumDrive;
import org.firstinspires.ftc.teamcode.library.hardware.actuators.Motor;
import org.firstinspires.ftc.teamcode.library.hardware.gamepad.Button;
import org.firstinspires.ftc.teamcode.library.hardware.gamepad.GpBindings;
import org.firstinspires.ftc.teamcode.library.hardware.gamepad.Stick;
import org.firstinspires.ftc.teamcode.library.robot.RobotConstants;

import java.util.function.DoubleUnaryOperator;
import java.util.Objects;

public class RobotDrive extends Mechanism {
    public enum FollowerMode {
        MANUAL,
        FOLLOWING,
        HOLDING,
        TURNING
    }

    public static class ROBOT_DRIVE_PARAMS implements Params<ROBOT_DRIVE_PARAMS> {
        public DoubleUnaryOperator DEFAULT_STICK_CURVE = DoubleUnaryOperator.identity();
        public boolean START_TELEOP_DRIVE_ON_INIT = true;
        public boolean RESET_STICK_CURVE_ON_INIT = true;
        public boolean UPDATE_FOLLOWER_ON_INIT = true;
        public double INITIAL_HEADING_OFFSET = 0;
        public Stick TRANSLATION_STICK = Stick.LEFT;
        public Stick ROTATION_STICK = Stick.RIGHT;
        public Tuple<Button, GpBindings.GamepadId> ZERO_HEADING_BUTTON = new Tuple<>(Button.BACK, GpBindings.GamepadId.ONE);

        public ROBOT_DRIVE_PARAMS() {}

        private ROBOT_DRIVE_PARAMS(ROBOT_DRIVE_PARAMS other) {
            DEFAULT_STICK_CURVE = other.DEFAULT_STICK_CURVE;
            START_TELEOP_DRIVE_ON_INIT = other.START_TELEOP_DRIVE_ON_INIT;
            RESET_STICK_CURVE_ON_INIT = other.RESET_STICK_CURVE_ON_INIT;
            UPDATE_FOLLOWER_ON_INIT = other.UPDATE_FOLLOWER_ON_INIT;
            INITIAL_HEADING_OFFSET = other.INITIAL_HEADING_OFFSET;
            TRANSLATION_STICK = other.TRANSLATION_STICK;
            ROTATION_STICK = other.ROTATION_STICK;
            ZERO_HEADING_BUTTON = other.ZERO_HEADING_BUTTON;
        }

        public ROBOT_DRIVE_PARAMS defaultStickCurve(DoubleUnaryOperator defaultStickCurve) {
            this.DEFAULT_STICK_CURVE = defaultStickCurve == null ? DoubleUnaryOperator.identity() : defaultStickCurve;
            return this;
        }

        public ROBOT_DRIVE_PARAMS startTeleOpDriveOnInit(boolean startTeleOpDriveOnInit) {
            this.START_TELEOP_DRIVE_ON_INIT = startTeleOpDriveOnInit;
            return this;
        }

        public ROBOT_DRIVE_PARAMS resetStickCurveOnInit(boolean resetStickCurveOnInit) {
            this.RESET_STICK_CURVE_ON_INIT = resetStickCurveOnInit;
            return this;
        }

        public ROBOT_DRIVE_PARAMS updateFollowerOnInit(boolean updateFollowerOnInit) {
            this.UPDATE_FOLLOWER_ON_INIT = updateFollowerOnInit;
            return this;
        }

        public ROBOT_DRIVE_PARAMS initialHeadingOffset(double initialHeadingOffset) {
            this.INITIAL_HEADING_OFFSET = initialHeadingOffset;
            return this;
        }

        public ROBOT_DRIVE_PARAMS translationStick(Stick translationStick) {
            this.TRANSLATION_STICK = translationStick == null ? Stick.LEFT : translationStick;
            return this;
        }

        public ROBOT_DRIVE_PARAMS rotationStick(Stick rotationStick) {
            this.ROTATION_STICK = rotationStick == null ? Stick.RIGHT : rotationStick;
            return this;
        }

        public ROBOT_DRIVE_PARAMS zeroHeadingButton(Button zeroHeadingButton, GpBindings.GamepadId gamepadId) {
            this.ZERO_HEADING_BUTTON = new Tuple<>(zeroHeadingButton, gamepadId);
            return this;
        }

        @Override
        public ROBOT_DRIVE_PARAMS copy() {
            return new ROBOT_DRIVE_PARAMS(this);
        }

        @Override
        public ROBOT_DRIVE_PARAMS updateFromBot(RobotConstants bot) {
            return this;
        }
    }

    public final ROBOT_DRIVE_PARAMS params;
    public final MecanumDrive mecanumDrive;
    private FollowerMode followerMode = FollowerMode.MANUAL;
    private ShotSolver shotSolver; // used for auto alignment
    private boolean holdAtPathEnd;

    public RobotDrive(HardwareMap map, RobotConstants bot) {
        super(map, bot);
        params = resolveParams(RobotDrive.class, new ROBOT_DRIVE_PARAMS());
        mecanumDrive = new MecanumDrive(map, bot, params);
        shotSolver = new ShotSolver();
    }

    @Override
    public void stop() {
        breakFollowing();
    }

    public void setStartingPose(Pose pose) {
        mecanumDrive.setStartingPose(pose);
    }

    public void setPose(Pose pose) {
        mecanumDrive.setPose(pose);
    }

    public void setTeleOpDrive(DriveVector vector) {
        enterManualMode();
        mecanumDrive.setTeleOpDrive(vector);
    }

    public void setTeleOpDrive(DriveVector vector, boolean fieldCentric) {
        enterManualMode();
        mecanumDrive.setTeleOpDrive(vector, fieldCentric);
    }

    public void startTeleOpDrive() {
        mecanumDrive.setTeleOpDrive(DriveVector.IDLE);
        mecanumDrive.startTeleOpDrive();
        setManualMode();
    }

    public void startTeleOpDrive(boolean useBrakeMode) {
        Follower follower = mecanumDrive.follower;
        follower.setTeleOpDrive(0, 0, 0, false);
        follower.startTeleOpDrive(useBrakeMode);
        if (mecanumDrive.useManualDrive) {
            mecanumDrive.startTeleOpDrive();
        }
        setManualMode();
    }

    public Pose getPose() {
        return mecanumDrive.getPose();
    }

    public Vector getVelocity() {
        return mecanumDrive.getVelocity();
    }

    public double getAngularVelocity() {
        return mecanumDrive.getAngularVelocity();
    }

    public Vector getAcceleration() {
        return mecanumDrive.getAcceleration();
    }

    public void zeroHeadingOffset() {
        mecanumDrive.zeroHeadingOffset();
    }

    public void resetHeading(double heading) {
        mecanumDrive.resetHeading(heading);
    }

    public void setStickCurve(DoubleUnaryOperator curve) {
        mecanumDrive.setStickCurve(curve);
    }

    public Motor[] getDriveMotors() {
        return mecanumDrive.getDriveMotors();
    }

    public void followPath(Path path) {
        mecanumDrive.follower.followPath(Objects.requireNonNull(path, "path"));
        startFollowing(mecanumDrive.follower.getConstants().isAutomaticHoldEnd());
    }

    public void followPath(Path path, boolean holdEnd) {
        mecanumDrive.follower.followPath(Objects.requireNonNull(path, "path"), holdEnd);
        startFollowing(holdEnd);
    }

    public void followPath(PathChain pathChain) {
        mecanumDrive.follower.followPath(Objects.requireNonNull(pathChain, "pathChain"));
        startFollowing(mecanumDrive.follower.getConstants().isAutomaticHoldEnd());
    }

    public void followPath(PathChain pathChain, boolean holdEnd) {
        mecanumDrive.follower.followPath(Objects.requireNonNull(pathChain, "pathChain"), holdEnd);
        startFollowing(holdEnd);
    }

    public void followPath(PathChain pathChain, double maxPower, boolean holdEnd) {
        mecanumDrive.follower.followPath(Objects.requireNonNull(pathChain, "pathChain"), maxPower, holdEnd);
        startFollowing(holdEnd);
    }

    public void holdPoint(BezierPoint point, double heading) {
        mecanumDrive.follower.holdPoint(Objects.requireNonNull(point, "point"), heading);
        followerMode = FollowerMode.HOLDING;
        holdAtPathEnd = false;
    }

    public void holdPoint(BezierPoint point, double heading, boolean useHoldScaling) {
        mecanumDrive.follower.holdPoint(Objects.requireNonNull(point, "point"), heading, useHoldScaling);
        followerMode = FollowerMode.HOLDING;
        holdAtPathEnd = false;
    }

    public void holdPoint(Pose pose) {
        mecanumDrive.follower.holdPoint(Objects.requireNonNull(pose, "pose"));
        followerMode = FollowerMode.HOLDING;
        holdAtPathEnd = false;
    }

    public void holdPoint(Pose pose, boolean useHoldScaling) {
        mecanumDrive.follower.holdPoint(Objects.requireNonNull(pose, "pose"), useHoldScaling);
        followerMode = FollowerMode.HOLDING;
        holdAtPathEnd = false;
    }

    public void turn(double radians) {
        mecanumDrive.follower.turn(radians);
        followerMode = FollowerMode.TURNING;
        holdAtPathEnd = false;
    }

    public void turn(double radians, boolean counterClockwise) {
        mecanumDrive.follower.turn(radians, counterClockwise);
        followerMode = FollowerMode.TURNING;
        holdAtPathEnd = false;
    }

    public void turnTo(double headingRadians) {
        mecanumDrive.follower.turnTo(headingRadians);
        followerMode = FollowerMode.TURNING;
        holdAtPathEnd = false;
    }

    @Deprecated
    public void turnToDegrees(double headingDegrees) {
        mecanumDrive.follower.turnToDegrees(headingDegrees);
        followerMode = FollowerMode.TURNING;
        holdAtPathEnd = false;
    }

    @Deprecated
    public void turnDegrees(double degrees, boolean isLeft) {
        mecanumDrive.follower.turnDegrees(degrees, isLeft);
        followerMode = FollowerMode.TURNING;
        holdAtPathEnd = false;
    }

    public void breakFollowing() {
        mecanumDrive.follower.breakFollowing();
        if (mecanumDrive.useManualDrive) {
            mecanumDrive.startTeleOpDrive();
        }
        setManualMode();
    }

    public FollowerMode getFollowerMode() {
        return followerMode;
    }

    public boolean isBusy() {
        return mecanumDrive.follower.isBusy();
    }

    public boolean isIdle() {
        return !isBusy()&& followerMode != FollowerMode.HOLDING;
    }

    public boolean isTurning() {
        return mecanumDrive.follower.isTurning();
    }

    public Follower getFollower() {
        return mecanumDrive.follower;
    }
    public boolean isUsingManualDrive() {
        return mecanumDrive.useManualDrive;
    }

    private DriveVector getDriveVector(GpBindings gpBindings) {
        Stick translationStick = gpBindings.getCommand(params.TRANSLATION_STICK);
        Stick rotationStick = gpBindings.getCommand(params.ROTATION_STICK);
        double forward = translationStick == null ? 0 : translationStick.getY();
        double strafe = translationStick == null ? 0 : translationStick.getX();
        double rotate = rotationStick == null ? 0 : rotationStick.getX();
        return new DriveVector(forward, strafe, rotate, getPose().getHeading());
    }

    @Override
    public void periodic(State state) {
        mecanumDrive.periodic();
        updateFollowerMode();
        if (state == null || state.getRobotMode() == RobotMode.TELEOP&&state.getGpBindings() == null) {
            return;
        }
        if (state.getRobotMode() == RobotMode.TELEOP) {
            Button zeroHeadingButton = state.getGpBindings().getCommand(params.ZERO_HEADING_BUTTON);
            Button cameraRelocalization;
            Button automatedIntakePath;
            Button shootAndAutoAlign = state.getGpBindings().getCommand(params.ZERO_HEADING_BUTTON);

            DriveVector driveVector = getDriveVector(state.getGpBindings());
            if (followerMode != FollowerMode.MANUAL && isBusy()) {
                return;
            } else if (followerMode != FollowerMode.MANUAL && isIdle()) {
                enterManualMode();
            }
            if (zeroHeadingButton != null && zeroHeadingButton.isPressed()) {
                zeroHeadingOffset();
            }
            if (followerMode==FollowerMode.MANUAL) setTeleOpDrive(driveVector);

            if (state.getRobot().getVision().getLatestResult().getFiducialResults() != null) {

            }

            if (shootAndAutoAlign.isPressed()) {
                ShotSolver.ShotContext context = ShotSolver.ShotContext.Builder.create().
                        pose(state.getRobot().getPoseCompensatedForVelocity())//.
                        //state(state.getRobot().getFieldUpdates().getCurrentHiveState())
                        .c;
                turn(shotSolver.calculateShot(context).getA());
            } else if (shootAndAutoAlign.wasJustReleased()) {
                enterManualMode(); // if no other automation
            }
        }
    }

    private void enterManualMode() {
        if (followerMode != FollowerMode.MANUAL || !mecanumDrive.follower.isTeleopDrive()) {
            startTeleOpDrive();
        }
    }

    private void startFollowing(boolean holdEnd) {
        followerMode = FollowerMode.FOLLOWING;
        holdAtPathEnd = holdEnd;
    }

    private void setManualMode() {
        followerMode = FollowerMode.MANUAL;
        holdAtPathEnd = false;
    }

    private void updateFollowerMode() {
        if (followerMode == FollowerMode.FOLLOWING && !mecanumDrive.follower.isBusy()) {
            followerMode = holdAtPathEnd ? FollowerMode.HOLDING : FollowerMode.MANUAL;
            holdAtPathEnd = false;
        } else if (followerMode == FollowerMode.TURNING && !mecanumDrive.follower.isTurning()) {
            followerMode = FollowerMode.HOLDING;
        }
    }

}
