package org.firstinspires.ftc.teamcode.library.robot;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

import org.firstinspires.ftc.teamcode.library.hardware.actuators.Direction;
import org.firstinspires.ftc.teamcode.library.hardware.leds.RGBIndicator;
import org.firstinspires.ftc.teamcode.library.hardware.webcam.Limelight;
import org.firstinspires.ftc.teamcode.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.MecanumDrive;

/**
 * A utility class to hold all of the robot constants for each bot
 * Not to be confused with RobotConstants, which is a base class for all constants
 */
@Configurable
public class RobotConfigurations {

    public static RobotConstants OldDECODERobotTesting() {
        return new RobotConstants()
                // DRIVE CONSTANTS
                .maxPower(0.9)
                .frontLeftMotorName("frontLeft")
                .frontRightMotorName("frontRight")
                .backLeftMotorName("backLeft")
                .backRightMotorName("backRight")
                .frontLeftMotorDirection(Direction.FORWARD)
                .frontRightMotorDirection(Direction.REVERSE)
                .backLeftMotorDirection(Direction.FORWARD)
                .backRightMotorDirection(Direction.REVERSE)
                .xVelocity(0)
                .yVelocity(0)
                // PINPOINT CONSTANTS
                .pinpointConstants(new PinpointConstants())
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
                .addMechanism(Intake.class)
                .addMechanism(Limelight.class)
                .addMechanism(RGBIndicator.class)
                // COMPATIBILITY
                .useFollowerDrive(true);
    }
}
