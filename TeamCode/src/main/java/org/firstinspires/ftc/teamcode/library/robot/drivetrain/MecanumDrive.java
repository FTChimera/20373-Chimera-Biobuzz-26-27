package org.firstinspires.ftc.teamcode.library.robot.drivetrain;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.commands.Commands;
import com.pedropathing.ivy.groups.Groups;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.configs.Blackboard;
import org.firstinspires.ftc.teamcode.library.DriveVector;
import org.firstinspires.ftc.teamcode.library.hardware.actuators.Motor;
import org.firstinspires.ftc.teamcode.library.robot.Driver;
import org.firstinspires.ftc.teamcode.library.robot.RobotConstants;
import org.firstinspires.ftc.teamcode.mechanisms.RobotDrive;
import org.firstinspires.ftc.teamcode.pedroPathing.CommandConverter;

import java.util.function.DoubleUnaryOperator;
import java.util.function.Supplier;

public class MecanumDrive {
    public final Follower follower;
    private final Motor[] driveMotors;
    public final boolean useManualDrive;
    public final RobotDrive.ROBOT_DRIVE_PARAMS params;
    private DoubleUnaryOperator stickCurve;

    public static Pose lastPose = new Pose(0, 0, 0);
    public double headingOffset;

    public MecanumDrive(HardwareMap map, RobotConstants bot) {
        this(map, bot, new RobotDrive.ROBOT_DRIVE_PARAMS().updateFromBot(bot));
    }

    public MecanumDrive(HardwareMap map, RobotConstants bot, RobotDrive.ROBOT_DRIVE_PARAMS params) {
        this.params = params.copy();
        this.follower = bot.createFollower(map);
        this.useManualDrive = !bot.USE_FOLLOWER_DRIVE;
        this.headingOffset = this.params.INITIAL_HEADING_OFFSET;
        this.driveMotors = useManualDrive ? createDriveMotors(map, bot) : new Motor[0];
        setStickCurve(DoubleUnaryOperator.identity());
        if (this.params.RESET_STICK_CURVE_ON_INIT) {
            resetStickCurve();
        }
        if (this.params.START_TELEOP_DRIVE_ON_INIT) {
            startTeleOpDrive();
        }
        if (this.params.UPDATE_FOLLOWER_ON_INIT) {
            follower.update();
        }
    }

    private Motor[] createDriveMotors(HardwareMap map, RobotConstants bot) {
        try {
            return new Motor[] {
                    createDriveMotor(map, bot.FRONT_LEFT_MOTOR_NAME, bot.FRONT_LEFT_MOTOR_DIRECTION),
                    createDriveMotor(map, bot.FRONT_RIGHT_MOTOR_NAME, bot.FRONT_RIGHT_MOTOR_DIRECTION),
                    createDriveMotor(map, bot.BACK_LEFT_MOTOR_NAME, bot.BACK_LEFT_MOTOR_DIRECTION),
                    createDriveMotor(map, bot.BACK_RIGHT_MOTOR_NAME, bot.BACK_RIGHT_MOTOR_DIRECTION)
            };
        } catch (RuntimeException e) {
            throw new IllegalStateException("Unable to initialize mecanum drive motors.", e);
        }
    }

    private Motor createDriveMotor(HardwareMap map, String hardwareMapName, org.firstinspires.ftc.teamcode.library.hardware.actuators.Direction direction) {
        try {
            return new Motor(map, hardwareMapName).setDirection(direction);
        } catch (RuntimeException e) {
            throw new IllegalStateException("Unable to initialize drive motor: " + hardwareMapName, e);
        }
    }

    private DriveVector shapeManualDriveVector(DriveVector driveVector, boolean fieldCentric, double robotHeading) {
        DriveVector vector = driveVector.copy();
        vector.applyStickCurve(getStickCurve());
        vector.forward = -vector.forward;
        if (fieldCentric) {
            vector.setRobotHeading(robotHeading);
            return vector.rotateFieldCentric();
        } else {
            return vector;
        }
    }

    private DriveVector shapeFollowerDriveVector(DriveVector driveVector) {
        DriveVector vector = driveVector.copy();
        vector.applyStickCurve(getStickCurve());
        return vector;
    }

    private void setMotorPowers(double[] motorPowers) {
        for (int i = 0; i < driveMotors.length; i++) {
            driveMotors[i].setPower(motorPowers[i]);
        }
    }

    public void setStartingPose(Pose pose) {
        follower.setStartingPose(pose);
    }

    public void periodic() {
        lastPose = follower.getPose();
        follower.update();
    }

    public void setTeleOpDrive(DriveVector driveVector, boolean fieldCentric) {
        if (useManualDrive) {
            DriveVector vector = shapeManualDriveVector(driveVector, fieldCentric, lastPose.getHeading() - headingOffset);
            setMotorPowers(vector.calculateDrivePowers());
        } else {
            DriveVector vector = shapeFollowerDriveVector(driveVector);
            follower.setTeleOpDrive(
                    -vector.forward, -vector.strafe, -vector.rotate, fieldCentric, fieldCentric ? headingOffset : 0
            );
        }
    }

    public void setTeleOpDrive(DriveVector vector) {
        Driver currentDriver = Blackboard.get(Blackboard.Caption.DRIVER) != null ? Blackboard.get(Blackboard.Caption.DRIVER).getValue() : Driver.DEFAULT;
        setTeleOpDrive(vector, currentDriver.USE_FIELD_CENTRIC);
    }

    public void startTeleOpDrive() {
        if (useManualDrive) {
            setMotorPowers(DriveVector.IDLE.calculateDrivePowers());
        } else {
            follower.startTeleopDrive();
            follower.setTeleOpDrive(0, 0, 0, false);
        }
    }

    public Pose getPose() {
        return follower.getPose();
    }

    public void resetHeading(double newHeading) {
        Pose currentPose = follower.getPose();
        follower.setPose(currentPose.setHeading(newHeading));
    }

    public void setHeadingOffset(double newHeading) {
        headingOffset = newHeading;
    }

    public void zeroHeadingOffset() {
        headingOffset = follower.getPose().getHeading();
    }

    public void setPose(Pose pose) {
        follower.setPose(pose);
    }

    public Vector getVelocity() {
        return follower.getVelocity();
    }

    public double getAngularVelocity() {
        return follower.getAngularVelocity();
    }

    public Vector getAcceleration() {
        return follower.getAcceleration();
    }

    public Command teleOp(Supplier<DriveVector> vector, boolean fieldCentric) {
        return Groups.sequential(
                CommandConverter.toCommand(this::startTeleOpDrive),
                Commands.infinite(() -> this.setTeleOpDrive(vector.get(), fieldCentric))
        );
    }

    public Command teleOp(Supplier<DriveVector> vector) {
        Driver currentDriver = Blackboard.get(Blackboard.Caption.DRIVER) != null ? Blackboard.get(Blackboard.Caption.DRIVER).getValue() : Driver.DEFAULT;
        return teleOp(vector, currentDriver.USE_FIELD_CENTRIC);
    }

    public Command startTeleOpDriveCommand() {
        return Commands.instant(this::startTeleOpDrive);
    }

    public Command setTeleOpDriveCommand(Supplier<DriveVector> vector, Supplier<Boolean> fieldCentric) {
        return Commands.infinite(() -> setTeleOpDrive(vector.get(), fieldCentric.get()));
    }

    public Command setTeleOpDriveCommand(Supplier<DriveVector> vector) {
        return Commands.infinite(() -> setTeleOpDrive(vector.get()));
    }

    public Command zeroHeadingOffsetCommand() {
        return Commands.instant(this::zeroHeadingOffset);
    }

    public void setStickCurve(DoubleUnaryOperator curve) {
        stickCurve = curve == null ? params.DEFAULT_STICK_CURVE : curve;
    }

    public void resetStickCurve() {
        setStickCurve(params.DEFAULT_STICK_CURVE);
    }

    public DoubleUnaryOperator getStickCurve() {
        return stickCurve == null ? DoubleUnaryOperator.identity() : stickCurve;
    }

    public Motor[] getDriveMotors() {
        return driveMotors;
    }
}
