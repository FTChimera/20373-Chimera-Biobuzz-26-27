package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.config.ConfigVar;
import com.pedropathing.follower.Follower;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

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

                c.resetMode.set(PinpointLocalizer.ResetMode.RESET_AND_RECALIBRATE_IMU);
            });

    public static ForesightConfig FORESIGHT_CONFIG = new ForesightConfig(
            f -> {
                f.maxPathSpeed.set(0.8);
                FOLLOWER_MAX_PATH_SPEED = f.maxPathSpeed;
                FOLLOWER_BRAKE_AT_END = f.brakeAtEnd; // is true
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
