package org.firstinspires.ftc.teamcode.library.robot;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * A utility class to hold all of the robot constants for each bot
 * Not to be confused with RobotConstants, which is a base class for all constants
 */
public class RobotConfigurations {
    public static RobotConstants starterBot() {
        return new RobotConstants()
                // DRIVE CONSTANTS
                .maxPower(0.9)
                .frontLeftMotorName("frontLeft")
                .frontRightMotorName("frontRight")
                .backLeftMotorName("backLeft")
                .backRightMotorName("backRight")
                .frontLeftMotorDirection(DcMotorSimple.Direction.FORWARD)
                .frontRightMotorDirection(DcMotorSimple.Direction.REVERSE)
                .backLeftMotorDirection(DcMotorSimple.Direction.FORWARD)
                .backRightMotorDirection(DcMotorSimple.Direction.REVERSE)
                .xVelocity(0)
                .yVelocity(0)
                // PINPOINT CONSTANTS
                .pinpointLocalizerName("pinpoint")
                .forwardPodYInch(0)
                .strafePodXInch(0)
                .pinpointForwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
                .pinpointStrafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
                // FOLLOWER CONSTANTS AND PATH CONSTRAINTS
                .tValueConstraint(0.99)
                .timeoutConstraint(100)
                .brakingStrength(1)
                .brakingStart(1)
                .predictiveBrakingCoefficients(0,0,0)
                .headingPIDFCoefficients(0,0,0,0)
                .centripetalScaling(0);
    }
}
