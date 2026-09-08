package org.firstinspires.ftc.teamcode.library.robot;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.library.hardware.actuators.Direction;
import org.firstinspires.ftc.teamcode.library.hardware.leds.RGBIndicator;
import org.firstinspires.ftc.teamcode.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.Vision;

/**
 * A utility class to hold all the robot constants for each bot
 * Not to be confused with RobotConstants, which is a base class for all constants
 */
public class RobotConfigurations {

    public static RobotConstants OldDECODERobotTesting() {
        // TODO DELETE THIS ONCE 1ST ROBOT BUILT
        return new RobotConstants()
                // DRIVE CONSTANTS
                .maxPower(0.9)
                .frontLeftMotorName("frontLeftMotor")
                .frontRightMotorName("frontRightMotor")
                .backLeftMotorName("backLeftMotor")
                .backRightMotorName("backRightMotor")
                .frontLeftMotorDirection(Direction.REVERSE)
                .frontRightMotorDirection(Direction.FORWARD)
                .backLeftMotorDirection(Direction.REVERSE)
                .backRightMotorDirection(Direction.FORWARD)
                .botWidth(18.0) // close enough doesnt matter
                .botLength(18.0) //close enough doesnt matter
                .xVelocity(82.28564164769932)
                .yVelocity(65.39811886764886)
                // IMU CONSTANTS
                .imuHardwareMapName("imu")
                .imuLogoFacingDirection(RevHubOrientationOnRobot.LogoFacingDirection.LEFT)
                .imuUsbFacingDirection(RevHubOrientationOnRobot.UsbFacingDirection.UP)
                // PINPOINT CONSTANTS
                .pinpointConstants(new PinpointConstants().forwardPodY(-0.3125)
                        .strafePodX(-3.5)
                        .distanceUnit(DistanceUnit.INCH)
                        .hardwareMapName("pinpoint")
                        .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
                        .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
                        .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED))
                // FOLLOWER CONSTANTS AND PATH CONSTRAINTS
                .tValueConstraint(0.99)
                .timeoutConstraint(100)
                .brakingStrength(1)
                .brakingStart(1.2)
                .driveConstants(new FollowerConstants()
                        .mass(11.4)
                        .forwardZeroPowerAcceleration(-41.031359746341444)
                        .lateralZeroPowerAcceleration(-71.62199686407405)
                        .translationalPIDFCoefficients(new com.pedropathing.control.PIDFCoefficients(0.067, 0, 0.05, 0.01))
                        .headingPIDFCoefficients(new com.pedropathing.control.PIDFCoefficients(0.9, 0, 0.08, 0.02))
                        .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.02,0,0.0001,3.5,0.00001))
                        .centripetalScaling(0.003))
                .headingPIDFCoefficients(0.067,0,0.05,0.01)
                .centripetalScaling(0.003)
                // MECHANISM USAGE
                .configureDriveMechanisms()
                .addMechanism(Intake.class, new Intake.INTAKE_PARAMS()
                        .hardwareMapName("intake")
                        .intakeMotorDirection(Direction.REVERSE))
                .addMechanism(Vision.class, new Vision.VISION_PARAMS())
                .addMechanism(RGBIndicator.class, new RGBIndicator.RGBIndicator_PARAMS()
                        .hardwareMapName("rgb")
                        .panelsName("RGB INDICATOR"))
                // COMPATIBILITY
                .useFollowerDrive(true);
    }

    public static RobotConstants FirstRobot() {
        return new RobotConstants()
                // DRIVE CONSTANTS
                .maxPower(0.4)
                .frontLeftMotorName("frontLeft")
                .frontRightMotorName("frontRight")
                .backLeftMotorName("backLeft")
                .backRightMotorName("backRight")
                .frontLeftMotorDirection(Direction.REVERSE)
                .frontRightMotorDirection(Direction.FORWARD)
                .backLeftMotorDirection(Direction.REVERSE)
                .backRightMotorDirection(Direction.FORWARD)
                .botWidth(17.008)
                .botLength(14.1)
                .xVelocity(0)
                .yVelocity(0)
                // IMU CONSTANTS
                .imuHardwareMapName("imu")
                .imuLogoFacingDirection(RevHubOrientationOnRobot.LogoFacingDirection.UP)
                .imuUsbFacingDirection(RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)
                // PINPOINT CONSTANTS
                .pinpointConstants(new PinpointConstants()
                        .hardwareMapName("pinpoint")
                        .forwardPodY(-0.8704944520484759)
                        .strafePodX(-7.450610543799215)
                        .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
                        .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED))
                // FOLLOWER CONSTANTS AND PATH CONSTRAINTS
                .tValueConstraint(0.99)
                .timeoutConstraint(100)
                .brakingStrength(1)
                .brakingStart(1)
                .driveConstants(new FollowerConstants())
                .headingPIDFCoefficients(0,0,0,0)
                .centripetalScaling(0)
                // MECHANISM USAGE
                .configureDriveMechanisms()
                .addMechanism(Intake.class, new Intake.INTAKE_PARAMS()
                        .hardwareMapName("intake")
                        .intakeMotorDirection(Direction.REVERSE))
                .addMechanism(Vision.class, new Vision.VISION_PARAMS())
                // COMPATIBILITY
                .useFollowerDrive(true);
    }
}
