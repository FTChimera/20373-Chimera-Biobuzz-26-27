package org.firstinspires.ftc.teamcode.library.robot;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.commands.Commands;
import com.pedropathing.ivy.pedro.PedroCommands;
import com.pedropathing.paths.PathChain;
import com.pedropathing.paths.PathConstraints;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.library.DriveVector;
import org.firstinspires.ftc.teamcode.library.game.State;
import org.firstinspires.ftc.teamcode.mechanisms.Robot;
import org.firstinspires.ftc.teamcode.mechanisms.Vision;

import java.util.Objects;
import java.util.function.DoubleUnaryOperator;
import java.util.function.Supplier;

/**
 * Builds Ivy commands that act on one profile-configured robot.
 */
public final class RobotIvyBuilder {
    private final Robot robot;

    public RobotIvyBuilder(Robot robot) {
        this.robot = Objects.requireNonNull(robot, "robot");
    }

    public Command periodic(State state) {
        return Commands.infinite(() -> robot.periodic(state));
    }

    public Command periodic(Supplier<State> stateSupplier) {
        Objects.requireNonNull(stateSupplier, "stateSupplier");
        return Commands.infinite(() -> robot.periodic(stateSupplier.get()));
    }

    public Command startRobot() {
        return Commands.instant(robot::start);
    }

    public Command stopRobot() {
        return Commands.instant(robot::stop);
    }

    public Command followPath(PathChain path) {
        return PedroCommands.follow(follower(), Objects.requireNonNull(path, "path"));
    }

    public Command followPath(PathChain path, boolean holdEnd) {
        return PedroCommands.follow(follower(), Objects.requireNonNull(path, "path"), holdEnd);
    }

    public Command followPath(PathChain path, boolean holdEnd, double maxPower) {
        return PedroCommands.follow(follower(), Objects.requireNonNull(path, "path"), holdEnd, maxPower);
    }

    public Command turnTo(double headingRadians) {
        return PedroCommands.turnTo(follower(), headingRadians);
    }

    public Command turnTo(double headingRadians, PathConstraints constraints) {
        return PedroCommands.turnTo(follower(), headingRadians, Objects.requireNonNull(constraints, "constraints"));
    }

    public Command hold() {
        return PedroCommands.hold(follower());
    }

    public Command hold(Pose pose) {
        return PedroCommands.hold(follower(), Objects.requireNonNull(pose, "pose"));
    }

    public Command hold(Pose pose, PathConstraints constraints) {
        return PedroCommands.hold(
                follower(),
                Objects.requireNonNull(pose, "pose"),
                Objects.requireNonNull(constraints, "constraints")
        );
    }

    public Command startIntake() {
        return Commands.instant(robot::startIntake);
    }

    public Command runIntake(double power) {
        return robot.requireIntake().setPowerCommand(() -> power);
    }

    public Command runIntake(Supplier<Double> powerSupplier) {
        return robot.requireIntake().setPowerCommand(Objects.requireNonNull(powerSupplier, "powerSupplier"));
    }

    public Command runIntakeHalfPower() {
        return robot.requireIntake().halfPower();
    }

    public Command reverseIntake() {
        return robot.requireIntake().reverseFullPower();
    }

    public Command stopIntake() {
        return robot.requireIntake().stop();
    }

    public Command setStartingPose(Pose pose) {
        return Commands.instant(() -> robot.setStartingPose(Objects.requireNonNull(pose, "pose")));
    }

    public Command setPose(Pose pose) {
        return Commands.instant(() -> robot.setPose(Objects.requireNonNull(pose, "pose")));
    }

    public Command startTeleOpDrive() {
        return Commands.instant(robot::startTeleOpDrive);
    }

    public Command setTeleOpDrive(DriveVector driveVector) {
        return Commands.instant(() -> robot.setTeleOpDrive(Objects.requireNonNull(driveVector, "driveVector")));
    }

    public Command setTeleOpDrive(DriveVector driveVector, boolean fieldCentric) {
        return Commands.instant(() -> robot.setTeleOpDrive(
                Objects.requireNonNull(driveVector, "driveVector"),
                fieldCentric
        ));
    }

    public Command setDriveStickCurve(DoubleUnaryOperator stickCurve) {
        return Commands.instant(() -> robot.setDriveStickCurve(Objects.requireNonNull(stickCurve, "stickCurve")));
    }

    public Command zeroDriveHeading() {
        return Commands.instant(robot::zeroDriveHeading);
    }

    public Command startVision(Vision.Pipeline pipeline) {
        return Commands.instant(() -> robot.startVision(Objects.requireNonNull(pipeline, "pipeline")));
    }

    public Command stopVision() {
        return Commands.instant(robot::stopVision);
    }

    public Command setVisionPipeline(Vision.Pipeline pipeline) {
        return Commands.instant(() -> robot.setVisionPipeline(Objects.requireNonNull(pipeline, "pipeline")));
    }

    public Command forceRelocalize() {
        return Commands.instant(robot::forceRelocalize);
    }

    private com.pedropathing.follower.Follower follower() {
        return robot.requireDrive().mecanumDrive.follower;
    }
}
