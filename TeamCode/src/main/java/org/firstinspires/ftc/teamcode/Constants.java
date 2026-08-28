package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.InvertedFTCCoordinates;
import com.pedropathing.geometry.CoordinateSystem;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.library.hardware.leds.RGBIndicator;
import org.firstinspires.ftc.teamcode.library.hardware.webcam.LogitechWebcam;
import org.firstinspires.ftc.teamcode.library.game.RobotMode;
import org.firstinspires.ftc.teamcode.library.robot.RobotConfigurations;
import org.firstinspires.ftc.teamcode.library.robot.RobotConstants;
import org.firstinspires.ftc.teamcode.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.Mechanism;
import org.firstinspires.ftc.teamcode.mechanisms.Robot;
import org.firstinspires.ftc.teamcode.mechanisms.RobotDrive;
import org.firstinspires.ftc.teamcode.mechanisms.Vision;

import java.util.Set;

@Configurable
public class Constants {
    /** https://ftc-docs.firstinspires.org/en/latest/game_specific_resources/field_coordinate_system/field-coordinate-system.html#coordinate-system
     * Look when game is released and change the constant */
    public static final CoordinateSystem FTC_COORDINATE_SYSTEM = InvertedFTCCoordinates.INSTANCE;
    public static final Pose RED_DEFAULT_START_POSE = new Pose(0, 0, 0);
    public static final Pose BLUE_DEFAULT_START_POSE = new Pose(0, 0, 0);
    public static final RobotConstants currentBot = RobotConfigurations.FirstRobot();

    public static Follower createFollower(HardwareMap hardwareMap) {
        return currentBot.createFollower(hardwareMap);
    }
    public static IMU createIMU(HardwareMap hardwareMap) {
        IMU imu = hardwareMap.get(IMU.class, currentBot.IMU_HARDWAREMAP_NAME);
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                currentBot.IMU_LOGO_FACING_DIRECTION,
                currentBot.IMU_USB_FACING_DIRECTION
        )));
        return imu;
    }
    public static RobotDrive createDrive(HardwareMap hardwareMap) {
        assertHasMechanism(RobotDrive.class);
        return currentBot.createMechanism(hardwareMap, RobotDrive.class);
    }

    public static Intake createIntake(HardwareMap hardwareMap) {
        assertHasMechanism(Intake.class);
        return currentBot.createMechanism(hardwareMap, Intake.class);
    }

    public static Vision createVision(HardwareMap hardwareMap) {
        assertHasMechanism(Vision.class);
        return currentBot.createMechanism(hardwareMap, Vision.class);
    }


    public static RGBIndicator createRGB(HardwareMap hardwareMap) {
        assertHasMechanism(RGBIndicator.class);
        return currentBot.createMechanism(hardwareMap, RGBIndicator.class);
    }

    public static LogitechWebcam createWebcam(HardwareMap hardwareMap) {
        return new LogitechWebcam(hardwareMap);
    }

    public static LogitechWebcam createWebcam(HardwareMap hardwareMap, LogitechWebcam.LOGITECH_WEBCAM_PARAMS params) {
        return new LogitechWebcam(hardwareMap, params);
    }

    public static Robot createRobot(HardwareMap hardwareMap) {
        return createRobot(hardwareMap, RobotMode.TELEOP);
    }

    public static Robot createRobot(HardwareMap hardwareMap, RobotMode robotMode) {
        return new Robot(hardwareMap, currentBot, robotMode);
    }

    public static <T> T createMechanism(HardwareMap hardwareMap, Class<T> mechClass) {
        assertHasMechanism((Class<? extends Mechanism>) mechClass);
        return currentBot.createMechanism(hardwareMap, mechClass);
    }

    public static Set<Class<? extends Mechanism>> getMechanisms() {
        return currentBot.Mechanisms;
    }

    public static boolean hasMechanism(Class<? extends Mechanism> mechClass) {
        return currentBot.Mechanisms.contains(mechClass);
    }

    public static void assertHasMechanism(Class<? extends Mechanism> mechClass){
        if (!hasMechanism(mechClass)) {
            throw new IllegalStateException("Mechanism not configured in robot: " + mechClass.getSimpleName());
        }
    }
}
