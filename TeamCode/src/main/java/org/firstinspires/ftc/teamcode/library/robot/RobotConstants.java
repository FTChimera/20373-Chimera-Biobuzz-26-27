package org.firstinspires.ftc.teamcode.library.robot;

import com.pedropathing.control.PredictiveBrakingCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.library.hardware.actuators.Direction;

/**
 * A utility base constants class to hold all Pedro pathing constants for a bot
 * Not to be confused with RobotConfigurations, which is a utility class to hold all the bot constants
 */
public class RobotConstants {
    public String FRONT_LEFT_MOTOR_NAME = "frontLeft";
    public String FRONT_RIGHT_MOTOR_NAME = "frontRight";
    public String BACK_LEFT_MOTOR_NAME = "backLeft";
    public String BACK_RIGHT_MOTOR_NAME = "backRight";
    public String PINPOINT_LOCALIZER_NAME = "pinpoint";

    public Direction FRONT_LEFT_MOTOR_DIRECTION = Direction.FORWARD;
    public Direction FRONT_RIGHT_MOTOR_DIRECTION = Direction.REVERSE;
    public Direction BACK_LEFT_MOTOR_DIRECTION = Direction.FORWARD;
    public Direction BACK_RIGHT_MOTOR_DIRECTION = Direction.REVERSE;
    public GoBildaPinpointDriver.EncoderDirection PINPOINT_FORWARD_ENCODER_DIRECTION = GoBildaPinpointDriver.EncoderDirection.FORWARD;
    public GoBildaPinpointDriver.EncoderDirection PINPOINT_STRAFE_ENCODER_DIRECTION = GoBildaPinpointDriver.EncoderDirection.FORWARD;
    public double FORWARD_POD_Y_INCH = 0;
    public double STRAFE_POD_X_INCH = 0;
    public double MAX_POWER = 0.9;
    public double X_VELOCITY = 0;
    public double Y_VELOCITY = 0;

    public double T_VALUE_CONSTRAINT = 0.99;
    public double TIMEOUT_CONSTRAINT = 100;
    public double BRAKING_STRENGTH = 1;
    public double BRAKING_START = 1;

    public double PREDICTIVE_BRAKING_COEFFICIENTS_PROPORTIONAL = 0;
    public double PREDICTIVE_BRAKING_COEFFICIENTS_LINEAR_BRAKING = 0;
    public double PREDICTIVE_BRAKING_COEFFICIENTS_QUADRATIC_FRICTION = 0;
    public double HEADING_PIDF_COEFFICIENTS_PROPORTIONAL = 0;
    public double HEADING_PIDF_COEFFICIENTS_INTEGRAL = 0;
    public double HEADING_PIDF_COEFFICIENTS_DERIVATIVE = 0;
    public double HEADING_PIDF_COEFFICIENTS_FEEDFORWARD = 0;
    public double CENTRIPETAL_SCALING = 0;

    public boolean USE_INTAKE = false;
    public boolean USE_LIMELIGHT = false;
    public boolean USE_LOGITECH_WEBCAM = false;

    public boolean FOLLOWER_TELEOP_DRIVE_COMPATIBLE = true; // Our team had issues with this last year, then finally got it to work.

    /** Default Constructor */
    public RobotConstants() {}

    public RobotConstants frontLeftMotorName(String name) {
        this.FRONT_LEFT_MOTOR_NAME = name;
        return this;
    }
    public RobotConstants frontRightMotorName(String name) {
        this.FRONT_RIGHT_MOTOR_NAME = name;
        return this;
    }
    public RobotConstants backLeftMotorName(String name) {
        this.BACK_LEFT_MOTOR_NAME = name;
        return this;
    }
    public RobotConstants backRightMotorName(String name) {
        this.BACK_RIGHT_MOTOR_NAME = name;
        return this;
    }
    public RobotConstants pinpointLocalizerName(String name) {
        this.PINPOINT_LOCALIZER_NAME = name;
        return this;
    }
    public RobotConstants frontLeftMotorDirection(Direction direction) {
        this.FRONT_LEFT_MOTOR_DIRECTION = direction;
        return this;
    }
    public RobotConstants frontRightMotorDirection(Direction direction) {
        this.FRONT_RIGHT_MOTOR_DIRECTION = direction;
        return this;
    }
    public RobotConstants backLeftMotorDirection(Direction direction) {
        this.BACK_LEFT_MOTOR_DIRECTION = direction;
        return this;
    }
    public RobotConstants backRightMotorDirection(Direction direction) {
        this.BACK_RIGHT_MOTOR_DIRECTION = direction;
        return this;
    }
    public RobotConstants pinpointForwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection direction) {
        this.PINPOINT_FORWARD_ENCODER_DIRECTION = direction;
        return this;
    }
    public RobotConstants pinpointStrafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection direction) {
        this.PINPOINT_STRAFE_ENCODER_DIRECTION = direction;
        return this;
    }
    public RobotConstants forwardPodYInch(double forwardPodYInch) {
        this.FORWARD_POD_Y_INCH = forwardPodYInch;
        return this;
    }
    public RobotConstants strafePodXInch(double strafePodXInch) {
        this.STRAFE_POD_X_INCH = strafePodXInch;
        return this;
    }
    public RobotConstants maxPower(double maxPower) {
        this.MAX_POWER = maxPower;
        return this;
    }
    public RobotConstants xVelocity(double xVelocity) {
        this.X_VELOCITY = xVelocity;
        return this;
    }
    public RobotConstants yVelocity(double yVelocity) {
        this.Y_VELOCITY = yVelocity;
        return this;
    }
    public RobotConstants tValueConstraint(double tValueConstraint) {
        this.T_VALUE_CONSTRAINT = tValueConstraint;
        return this;
    }
    public RobotConstants timeoutConstraint(double timeoutConstraint) {
        this.TIMEOUT_CONSTRAINT = timeoutConstraint;
        return this;
    }
    public RobotConstants brakingStrength(double brakingStrength) {
        this.BRAKING_STRENGTH = brakingStrength;
        return this;
    }
    public RobotConstants brakingStart(double brakingStart) {
        this.BRAKING_START = brakingStart;
        return this;
    }
    public RobotConstants predictiveBrakingCoefficients(double proportional, double linearBraking, double quadraticFriction) {
        this.PREDICTIVE_BRAKING_COEFFICIENTS_PROPORTIONAL = proportional;
        this.PREDICTIVE_BRAKING_COEFFICIENTS_LINEAR_BRAKING = linearBraking;
        this.PREDICTIVE_BRAKING_COEFFICIENTS_QUADRATIC_FRICTION = quadraticFriction;
        return this;
    }
    public RobotConstants headingPIDFCoefficients(double proportional, double integral, double derivative, double feedforward) {
        this.HEADING_PIDF_COEFFICIENTS_PROPORTIONAL = proportional;
        this.HEADING_PIDF_COEFFICIENTS_INTEGRAL = integral;
        this.HEADING_PIDF_COEFFICIENTS_DERIVATIVE = derivative;
        this.HEADING_PIDF_COEFFICIENTS_FEEDFORWARD = feedforward;
        return this;
    }
    public RobotConstants centripetalScaling(double centripetalScaling) {
        this.CENTRIPETAL_SCALING = centripetalScaling;
        return this;
    }

    public RobotConstants useIntake(boolean useIntake) {
        this.USE_INTAKE = useIntake;
        return this;
    }
    public RobotConstants useLimelight(boolean useLimelight) {
        this.USE_LIMELIGHT = useLimelight;
        return this;
    }
    public RobotConstants useLogitechWebcam(boolean useLogitechWebcam) {
        this.USE_LOGITECH_WEBCAM = useLogitechWebcam;
        return this;
    }
    public RobotConstants followerTeleopDriveCompatible(boolean followerTeleopDriveCompatible) {
        this.FOLLOWER_TELEOP_DRIVE_COMPATIBLE = followerTeleopDriveCompatible;
        return this;
    }
    public Follower createFollower(HardwareMap map) {
        // Pedro Pathing Follower configuration
        FollowerConstants followerConstants = new FollowerConstants()
                .predictiveBrakingCoefficients(new PredictiveBrakingCoefficients(PREDICTIVE_BRAKING_COEFFICIENTS_PROPORTIONAL,PREDICTIVE_BRAKING_COEFFICIENTS_LINEAR_BRAKING,PREDICTIVE_BRAKING_COEFFICIENTS_QUADRATIC_FRICTION))
                .headingPIDFCoefficients(new com.pedropathing.control.PIDFCoefficients(HEADING_PIDF_COEFFICIENTS_PROPORTIONAL, HEADING_PIDF_COEFFICIENTS_INTEGRAL, HEADING_PIDF_COEFFICIENTS_DERIVATIVE, HEADING_PIDF_COEFFICIENTS_FEEDFORWARD))
                .centripetalScaling(CENTRIPETAL_SCALING);
        MecanumConstants mecanumConstants = new MecanumConstants()
                .maxPower(MAX_POWER)
                .leftFrontMotorName(FRONT_LEFT_MOTOR_NAME)
                .rightFrontMotorName(FRONT_RIGHT_MOTOR_NAME)
                .leftRearMotorName(BACK_LEFT_MOTOR_NAME)
                .rightRearMotorName(BACK_RIGHT_MOTOR_NAME)
                .leftFrontMotorDirection(FRONT_LEFT_MOTOR_DIRECTION.toFtcSdk())
                .rightFrontMotorDirection(FRONT_RIGHT_MOTOR_DIRECTION.toFtcSdk())
                .leftRearMotorDirection(BACK_LEFT_MOTOR_DIRECTION.toFtcSdk())
                .rightRearMotorDirection(BACK_RIGHT_MOTOR_DIRECTION.toFtcSdk())
                .xVelocity(X_VELOCITY)
                .yVelocity(Y_VELOCITY);

        PinpointConstants pinpointConstants = new PinpointConstants()
                .forwardPodY(FORWARD_POD_Y_INCH)
                .strafePodX(STRAFE_POD_X_INCH)
                .distanceUnit(DistanceUnit.INCH)
                .hardwareMapName(PINPOINT_LOCALIZER_NAME)
                .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
                .forwardEncoderDirection(PINPOINT_FORWARD_ENCODER_DIRECTION)
                .strafeEncoderDirection(PINPOINT_STRAFE_ENCODER_DIRECTION);
        PathConstraints pathConstraints = new PathConstraints(T_VALUE_CONSTRAINT, TIMEOUT_CONSTRAINT, BRAKING_STRENGTH, BRAKING_START);

        return new FollowerBuilder(followerConstants, map)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(mecanumConstants)
                .pinpointLocalizer(pinpointConstants)
                .build();
    }
}
