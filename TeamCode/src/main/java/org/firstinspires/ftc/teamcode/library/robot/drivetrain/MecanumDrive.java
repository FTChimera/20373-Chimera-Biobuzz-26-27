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
import org.firstinspires.ftc.teamcode.library.robot.Driver;
import org.firstinspires.ftc.teamcode.library.robot.RobotConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.CommandConverter;

import java.util.function.DoubleUnaryOperator;
import java.util.function.Supplier;

public class MecanumDrive {
    public static class MECANUM_DRIVE_PARAMS {
        public boolean RESET_STICK_CURVE_ON_INIT = true;
        public boolean UPDATE_FOLLOWER_ON_INIT = true;
        public double INITIAL_HEADING_OFFSET = 0;

        public MECANUM_DRIVE_PARAMS() {}

        private MECANUM_DRIVE_PARAMS(MECANUM_DRIVE_PARAMS other) {
            RESET_STICK_CURVE_ON_INIT = other.RESET_STICK_CURVE_ON_INIT;
            UPDATE_FOLLOWER_ON_INIT = other.UPDATE_FOLLOWER_ON_INIT;
            INITIAL_HEADING_OFFSET = other.INITIAL_HEADING_OFFSET;
        }

        public MECANUM_DRIVE_PARAMS resetStickCurveOnInit(boolean resetStickCurveOnInit) {
            this.RESET_STICK_CURVE_ON_INIT = resetStickCurveOnInit;
            return this;
        }

        public MECANUM_DRIVE_PARAMS updateFollowerOnInit(boolean updateFollowerOnInit) {
            this.UPDATE_FOLLOWER_ON_INIT = updateFollowerOnInit;
            return this;
        }

        public MECANUM_DRIVE_PARAMS initialHeadingOffset(double initialHeadingOffset) {
            this.INITIAL_HEADING_OFFSET = initialHeadingOffset;
            return this;
        }

        public MECANUM_DRIVE_PARAMS copy() {
            return new MECANUM_DRIVE_PARAMS(this);
        }
    }

    public final Follower follower;
    public final Drivetrain drivetrain;
    public final boolean useManualDrive;
    public final MECANUM_DRIVE_PARAMS params;

    public static Pose lastPose = new Pose(0, 0, 0);
    public double headingOffset = 0;

    public MecanumDrive(HardwareMap map, RobotConstants bot) {
        this(map, bot, new MECANUM_DRIVE_PARAMS(), new Drivetrain(map, bot));
    }

    public MecanumDrive(HardwareMap map, RobotConstants bot, MECANUM_DRIVE_PARAMS params, Drivetrain drivetrain) {
        this.params = params.copy();
        this.follower = bot.createFollower(map);
        this.useManualDrive = !bot.USE_FOLLOWER_DRIVE;
        this.drivetrain = drivetrain;
        this.headingOffset = this.params.INITIAL_HEADING_OFFSET;
        if (this.params.RESET_STICK_CURVE_ON_INIT) {
            resetStickCurve();
        }
        if (this.params.UPDATE_FOLLOWER_ON_INIT) {
            follower.update();
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
            drivetrain.setTeleOpDrive(driveVector, fieldCentric, lastPose.getHeading() - headingOffset);
        } else {
            driveVector.applyStickCurve(getStickCurve());
            follower.setTeleOpDrive(
                    -driveVector.forward, -driveVector.strafe, -driveVector.rotate, fieldCentric, fieldCentric ? headingOffset : 0
            );
        }
    }

    public void setTeleOpDrive(DriveVector vector) {
        Driver currentDriver = Blackboard.get(Blackboard.Caption.DRIVER) != null ? Blackboard.get(Blackboard.Caption.DRIVER).getValue() : Driver.DEFAULT;
        setTeleOpDrive(vector, currentDriver.USE_FIELD_CENTRIC);
    }

    public void startTeleOpDrive() {
        if (useManualDrive) {
            drivetrain.startTeleOpDrive();
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

    public Command zeroHeadingOffsetCommand() {
        return Commands.instant(this::zeroHeadingOffset);
    }

    public void setStickCurve(DoubleUnaryOperator curve) {
        drivetrain.setStickCurve(curve);
    }

    public void resetStickCurve() {
        drivetrain.setStickCurve(DoubleUnaryOperator.identity());
    }

    public DoubleUnaryOperator getStickCurve() {
        return drivetrain.params.DEFAULT_STICK_CURVE == null ? DoubleUnaryOperator.identity() : drivetrain.params.DEFAULT_STICK_CURVE;
    }
}
