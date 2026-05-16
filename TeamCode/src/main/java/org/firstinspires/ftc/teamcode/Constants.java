package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.control.PredictiveBrakingCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.custom.LimelightPinpointLocalizer;

@Configurable
public class Constants {

    public static final String FRONT_LEFT_MOTOR_NAME = "frontLeft";
    public static final String FRONT_RIGHT_MOTOR_NAME = "frontRight";
    public static final String BACK_LEFT_MOTOR_NAME = "backLeft";
    public static final String BACK_RIGHT_MOTOR_NAME = "backRight";
    public static final String INTAKE_MOTOR_NAME = "intake";
    public static final String LIMELIGHT_CAMERA_NAME = "limelight";
    public static final String PINPOINT_LOCALIZER_NAME = "pinpoint";

    public static final DcMotorSimple.Direction FRONT_LEFT_MOTOR_DIRECTION = DcMotorSimple.Direction.FORWARD;
    public static final DcMotorSimple.Direction FRONT_RIGHT_MOTOR_DIRECTION = DcMotorSimple.Direction.REVERSE;
    public static final DcMotorSimple.Direction BACK_LEFT_MOTOR_DIRECTION = DcMotorSimple.Direction.FORWARD;
    public static final DcMotorSimple.Direction BACK_RIGHT_MOTOR_DIRECTION = DcMotorSimple.Direction.REVERSE;
    public static final DcMotorSimple.Direction INTAKE_MOTOR_DIRECTION = DcMotorSimple.Direction.REVERSE;
    public static final GoBildaPinpointDriver.EncoderDirection PINPOINT_FORWARD_ENCODER_DIRECTION = GoBildaPinpointDriver.EncoderDirection.FORWARD;
    public static final GoBildaPinpointDriver.EncoderDirection PINPOINT_STRAFE_ENCODER_DIRECTION = GoBildaPinpointDriver.EncoderDirection.FORWARD;

    public static final Pose RED_DEFAULT_START_POSE = new Pose(0, 0, 0);
    public static final Pose BLUE_DEFAULT_START_POSE = new Pose(0, 0, 0);
    public static final boolean DRIVE_FIELD_RELATIVE = false;
    public static final double AUTO_MAX_POWER = 0.9;

    // Pedro Pathing Follower configuration
    public static FollowerConstants followerConstants = new FollowerConstants()
            .predictiveBrakingCoefficients(new PredictiveBrakingCoefficients(0,0,0))
            .headingPIDFCoefficients(new com.pedropathing.control.PIDFCoefficients(0, 0, 0, 0))
            .centripetalScaling(0);
    public static MecanumConstants mecanumConstants = new MecanumConstants()
            .maxPower(AUTO_MAX_POWER)
            .leftFrontMotorName(FRONT_LEFT_MOTOR_NAME)
            .rightFrontMotorName(FRONT_RIGHT_MOTOR_NAME)
            .leftRearMotorName(BACK_LEFT_MOTOR_NAME)
            .rightRearMotorName(BACK_RIGHT_MOTOR_NAME)
            .leftFrontMotorDirection(FRONT_LEFT_MOTOR_DIRECTION)
            .rightFrontMotorDirection(FRONT_RIGHT_MOTOR_DIRECTION)
            .leftRearMotorDirection(BACK_LEFT_MOTOR_DIRECTION)
            .rightRearMotorDirection(BACK_RIGHT_MOTOR_DIRECTION)
            .xVelocity(0)
            .yVelocity(0);

    // Don't do custom localizer in case camera is used for color detection in autonomous
    public static PinpointConstants pinpointConstants = new PinpointConstants()
            .forwardPodY(0)
            .strafePodX(0)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName(PINPOINT_LOCALIZER_NAME)
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(PINPOINT_FORWARD_ENCODER_DIRECTION)
            .strafeEncoderDirection(PINPOINT_STRAFE_ENCODER_DIRECTION);
    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static Follower pinpointFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(mecanumConstants)
                .pinpointLocalizer(pinpointConstants)
                .build();
    }

    // Make sure color detection is not used if using this localizer
    /*
    public static Follower customLocalizerFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(mecanumConstants)
                .setLocalizer(new LimelightPinpointLocalizer(pinpointConstants, hardwareMap, LIMELIGHT_CAMERA_NAME, 0))
                .build();
    }
     */
}
