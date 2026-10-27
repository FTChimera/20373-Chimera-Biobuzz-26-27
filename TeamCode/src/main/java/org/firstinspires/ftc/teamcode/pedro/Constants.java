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
    public static Tuple<String, Direction> BACK_LEFT_MOTOR = new Tuple<>("backLeft", Direction.REVERSE);
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

    public static PinpointConfig PINPOINT_CONFIG = new PinpointConfig(
            c -> {
                c.name.set("pinpoint");
                c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
                c.xPodOffset.set(-0.9598076437401959);
                c.yPodOffset.set(-6.778503177672859);
                c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
                c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
                c.globalDistanceUnit.set(DistanceUnit.INCH);
                c.offsetUnits.set(DistanceUnit.INCH);
                c.resetMode.set(PinpointLocalizer.ResetMode.RESET_AND_RECALIBRATE_IMU);
            });

    public static ForesightConfig FORESIGHT_CONFIG = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.25393523777407717);
                Controller secondaryTranslationalForward = Controller.proportional(0.093822285083002);
                Controller primaryTranslationalLateral = Controller.proportional(0.34211089068822603);
                Controller secondaryTranslationalLateral = Controller.proportional(0.1264008327379413);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.012646681150676874));
                c.brake.set(Controller.proportionalFeedforward(0.010749678978075342));

                c.headingFeedback.set(Controller.proportional(3.2355359581547116));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.048720657697875876, 0.007348353148259572));

                c.linearBrakeCoefficients.set(Matrix.diag(0.07910109428104556, 0.06292995560631676));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.0020346280054097223, 0.0018481479677086117));

                c.maxAchievableForwardVelocity.set(75.46293364655632);
                c.maxAchievableStrafeVelocity.set(61.97638820235382);
                c.naturalForwardDeceleration.set(40.570781365019975);
                c.naturalStrafeDeceleration.set(73.55783046488493);

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
}
