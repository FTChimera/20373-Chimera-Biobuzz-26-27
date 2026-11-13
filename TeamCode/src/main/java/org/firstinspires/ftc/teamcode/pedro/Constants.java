package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.config.ConfigVar;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.util.Tuple;
import org.firstinspires.ftc.teamcode.util.Direction;

public class Constants {
    public static Tuple<String, Direction> FRONT_LEFT_MOTOR = new Tuple<>("frontLeft", Direction.REVERSE);
    public static Tuple<String, Direction> BACK_LEFT_MOTOR = new Tuple<>("backLeft", Direction.FORWARD);
    public static Tuple<String, Direction> FRONT_RIGHT_MOTOR = new Tuple<>("frontRight", Direction.FORWARD);
    public static Tuple<String, Direction> BACK_RIGHT_MOTOR = new Tuple<>("backRight", Direction.FORWARD);

    public static final String COMPETITION_GROUP = "                                             competition";
    public static ConfigVar<Boolean> FOLLOWER_MANUAL_BRAKE_MODE = null;
    public static ConfigVar<Double> FOLLOWER_MAX_PATH_SPEED = null;
    public static ConfigVar<Boolean> FOLLOWER_BRAKE_AT_END = null;
    public static MecanumConfig MECANUM_CONFIG = new MecanumConfig(
            mecanumConfig -> {
                mecanumConfig.frontLeftName.set(FRONT_LEFT_MOTOR.getA());
                mecanumConfig.backLeftName.set(BACK_LEFT_MOTOR.getA());
                mecanumConfig.frontRightName.set(FRONT_RIGHT_MOTOR.getA());
                mecanumConfig.backRightName.set(BACK_RIGHT_MOTOR.getA());

                mecanumConfig.frontLeftDirection.set(FRONT_LEFT_MOTOR.getB().toMotor());
                mecanumConfig.backLeftDirection.set(BACK_LEFT_MOTOR.getB().toMotor());
                mecanumConfig.frontRightDirection.set(FRONT_RIGHT_MOTOR.getB().toMotor());
                mecanumConfig.backRightDirection.set(BACK_RIGHT_MOTOR.getB().toMotor());

                mecanumConfig.manualBrakeMode.set(false);
                FOLLOWER_MANUAL_BRAKE_MODE = mecanumConfig.manualBrakeMode;
            }
    );

    public static EKFPinpointConfig PINPOINT_CONFIG = new EKFPinpointConfig(
            c -> {
                c.name.set("pinpoint");
                c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
                c.xPodOffset.set(-1.0052656939649207);
                c.yPodOffset.set(-7.002453541192483);
                c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
                c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
                c.globalDistanceUnit.set(DistanceUnit.INCH);
                c.offsetUnits.set(DistanceUnit.INCH);
                c.resetMode.set(PinpointLocalizer.ResetMode.RESET_AND_RECALIBRATE_IMU);

                c.initialPositionStandardDeviation.set(0.5);
                c.initialHeadingStandardDeviation.set(Math.toRadians(5.0));
                c.initialVelocityStandardDeviation.set(1.0);
                c.initialAngularVelocityStandardDeviation.set(0.1);
                
                c.linearAccelerationStandardDeviation.set(72.0);
                c.angularAccelerationStandardDeviation.set(8.0);
                c.positionMeasurementStandardDeviation.set(0.1);
                c.headingMeasurementStandardDeviation.set(Math.toRadians(1.0));

            });

    public static ForesightConfig FORESIGHT_CONFIG = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.29722722419657965);
                Controller secondaryTranslationalForward = Controller.proportional(0.10981751728293468);
                Controller primaryTranslationalLateral = Controller.proportional(1.9274661674592535);
                Controller secondaryTranslationalLateral = Controller.proportional(0.7121472460316581);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.01367709053108971));
                c.brake.set(Controller.proportionalFeedforward(0.011625526951426253));

                c.headingFeedback.set(Controller.proportional(4.309966151982717));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.04784676414619013, 0.0062965701809444795));

                c.linearBrakeCoefficients.set(Matrix.diag(0.10796558378583486, 0.08951549150090347));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.0012960898400324297, 8.258264751519653E-4));

                c.maxAchievableForwardVelocity.set(71.99251198931624);
                c.maxAchievableStrafeVelocity.set(57.73912192155395);
                c.naturalForwardDeceleration.set(29.439881390564473);
                c.naturalStrafeDeceleration.set(68.19193697145127);

                c.maxPathSpeed.set(1.0);
                c.brakeAtEnd.set(true);

                FOLLOWER_MAX_PATH_SPEED = c.maxPathSpeed;
                FOLLOWER_BRAKE_AT_END = c.brakeAtEnd; // is true
            });


    public static IMU createIMU(HardwareMap hardwareMap) {
        IMU imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP
        )));
        return imu;
    }

    public static Follower createFollowerPedro(HardwareMap hardwareMap) {
        return new Follower(
                new PinpointLocalizer(hardwareMap, Constants.PINPOINT_CONFIG),
                new Mecanum(hardwareMap, Constants.MECANUM_CONFIG),
                new Foresight(Constants.FORESIGHT_CONFIG)
        );
    }

    public static org.firstinspires.ftc.teamcode.pedro.Follower createFollower(HardwareMap hardwareMap) {
        return new org.firstinspires.ftc.teamcode.pedro.Follower(
                new PinpointLocalizer(hardwareMap, Constants.PINPOINT_CONFIG),
                new Mecanum(hardwareMap, Constants.MECANUM_CONFIG),
                new Foresight(Constants.FORESIGHT_CONFIG)
        );
    }

    public static org.firstinspires.ftc.teamcode.pedro.Follower createEKFFollower(HardwareMap hardwareMap) {
        return new org.firstinspires.ftc.teamcode.pedro.Follower(
                new EKFPinpoint(hardwareMap, Constants.PINPOINT_CONFIG),
                new Mecanum(hardwareMap, Constants.MECANUM_CONFIG),
                new Foresight(Constants.FORESIGHT_CONFIG)
        );
    }
}
