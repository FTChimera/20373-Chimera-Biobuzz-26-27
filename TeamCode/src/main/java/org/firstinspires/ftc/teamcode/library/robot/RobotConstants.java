package org.firstinspires.ftc.teamcode.library.robot;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.library.control.DoublePIDFCoeffs;
import org.firstinspires.ftc.teamcode.library.hardware.actuators.Direction;
import org.firstinspires.ftc.teamcode.mechanisms.MecanumDrive;
import org.firstinspires.ftc.teamcode.mechanisms.Mechanism;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * A utility base constants class to hold all Pedro pathing constants for a bot
 * Not to be confused with RobotConfigurations, which is a utility class to hold all the bot constants
 */
public class RobotConstants {
    public String FRONT_LEFT_MOTOR_NAME = "frontLeft";
    public String FRONT_RIGHT_MOTOR_NAME = "frontRight";
    public String BACK_LEFT_MOTOR_NAME = "backLeft";
    public String BACK_RIGHT_MOTOR_NAME = "backRight";

    public Direction FRONT_LEFT_MOTOR_DIRECTION = Direction.FORWARD;
    public Direction FRONT_RIGHT_MOTOR_DIRECTION = Direction.REVERSE;
    public Direction BACK_LEFT_MOTOR_DIRECTION = Direction.FORWARD;
    public Direction BACK_RIGHT_MOTOR_DIRECTION = Direction.REVERSE;
    public double MAX_POWER = 0.9;
    public double X_VELOCITY = 0;
    public double Y_VELOCITY = 0;

    public PinpointConstants PINPOINT_LOCALIZER_CONSTANTS = new PinpointConstants();

    public double T_VALUE_CONSTRAINT = 0.99;
    public double TIMEOUT_CONSTRAINT = 100;
    public double BRAKING_STRENGTH = 1;
    public double BRAKING_START = 1;

    public FollowerConstants DRIVE_CONSTANTS = new FollowerConstants();
    public DoublePIDFCoeffs HEADING_PIDF_COEFFICIENTS = new DoublePIDFCoeffs(0,0,0,0);
    public double CENTRIPETAL_SCALING = 0;

    public List<Class<? extends Mechanism>> Mechanisms = new ArrayList<>(Collections.singleton(MecanumDrive.class));

    public boolean USE_FOLLOWER_DRIVE = true; // Our team had issues with this last year, then finally got it to work.

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
    public RobotConstants pinpointConstants(PinpointConstants pinpointConstants) {
        this.PINPOINT_LOCALIZER_CONSTANTS = pinpointConstants;
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
    public RobotConstants driveConstants(FollowerConstants fConstants) {
        this.DRIVE_CONSTANTS = fConstants;
        return this;
    }
    public RobotConstants headingPIDFCoefficients(double proportional, double integral, double derivative, double feedforward) {
        this.HEADING_PIDF_COEFFICIENTS = new DoublePIDFCoeffs(proportional,integral,derivative,feedforward);
        return this;
    }
    public RobotConstants centripetalScaling(double centripetalScaling) {
        this.CENTRIPETAL_SCALING = centripetalScaling;
        return this;
    }
    public RobotConstants addMechanism(Class<? extends Mechanism> mechanismClass) {
        Mechanisms.add(mechanismClass);
        return this;
    }
    public RobotConstants useFollowerDrive(boolean useFollowerDrive) {
        this.USE_FOLLOWER_DRIVE = useFollowerDrive;
        return this;
    }
    public Follower createFollower(HardwareMap map) {
        // Pedro Pathing Follower configuration
        FollowerConstants followerConstants = DRIVE_CONSTANTS
                .headingPIDFCoefficients(HEADING_PIDF_COEFFICIENTS.getAsPedroPathing())
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

        PinpointConstants pinpointConstants = PINPOINT_LOCALIZER_CONSTANTS
                .distanceUnit(DistanceUnit.INCH)
                .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        PathConstraints pathConstraints = new PathConstraints(T_VALUE_CONSTRAINT, TIMEOUT_CONSTRAINT, BRAKING_STRENGTH, BRAKING_START);

        return new FollowerBuilder(followerConstants, map)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(mecanumConstants)
                .pinpointLocalizer(pinpointConstants)
                .build();
    }
    public <T> T createMechanism(HardwareMap map, Class<? extends T> mechanismClass) {
        try {
            if (!(Mechanisms.contains(mechanismClass))) {
                throw new IllegalArgumentException("Mechanism " + mechanismClass.getSimpleName() + " is not configured for this robot.");
            }
            return mechanismClass.cast(mechanismClass.getConstructor(HardwareMap.class, RobotConstants.class).newInstance(map, this));
        } catch (Exception e) {
            throw new RuntimeException("Error creating mechanism: " + mechanismClass.getSimpleName(), e);
        }
    }
}
