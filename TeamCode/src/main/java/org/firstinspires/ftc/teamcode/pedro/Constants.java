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
            f -> {
                Controller primaryTranslationalForward = Controller.proportional(0.2590734236566328);
                Controller secondaryTranslationalForward = Controller.proportional(0.09572070747175095);
                Controller primaryTranslationalLateral = Controller.proportional(0.3229333883091061);
                Controller secondaryTranslationalLateral = Controller.proportional(0.11931525804115767);

                f.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                f.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                f.coast.set(Controller.proportionalFeedforward(0.011724128037577176));
                f.brake.set(Controller.proportionalFeedforward(0.0099655088319406));

                f.headingFeedback.set(Controller.proportional(3.52768815337747));
                f.headingBrakeCoefficients.set(Vector2D.cartesian(0.04935565176248614, 0.0068564035845885525));

                f.linearBrakeCoefficients.set(Matrix.diag(0.07755767657551609, 0.043645391664046206));
                f.quadraticBrakeCoefficients.set(Matrix.diag(0.001692160003567388, 0.00228994514980421));

                f.maxAchievableForwardVelocity.set(81.1464712284475);
                f.maxAchievableStrafeVelocity.set(72.07259558080416);
                f.naturalForwardDeceleration.set(29.55621578414096);
                f.naturalStrafeDeceleration.set(56.068908924605985);

                f.maxPathSpeed.set(1.0);
                f.brakeAtEnd.set(true);

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
