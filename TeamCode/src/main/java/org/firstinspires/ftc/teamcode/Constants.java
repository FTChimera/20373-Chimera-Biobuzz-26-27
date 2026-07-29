package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.library.hardware.leds.RGBIndicator;
import org.firstinspires.ftc.teamcode.library.hardware.webcam.Limelight;
import org.firstinspires.ftc.teamcode.library.hardware.webcam.LogitechWebcam;
import org.firstinspires.ftc.teamcode.library.robot.RobotConfigurations;
import org.firstinspires.ftc.teamcode.library.robot.RobotConstants;
import org.firstinspires.ftc.teamcode.mechanisms.Drivetrain;
import org.firstinspires.ftc.teamcode.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.MecanumDrive;
import org.firstinspires.ftc.teamcode.mechanisms.Mechanism;
import org.firstinspires.ftc.teamcode.mechanisms.Slides1;

import java.util.Set;

@Configurable
public class Constants {
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
    public static MecanumDrive createTeleOpFollower(HardwareMap hardwareMap) {
        assertHasMechanism(MecanumDrive.class);
        return currentBot.createMechanism(hardwareMap, MecanumDrive.class);
    }

    public static Intake createIntake(HardwareMap hardwareMap) {
        assertHasMechanism(Intake.class);
        return currentBot.createMechanism(hardwareMap, Intake.class);
    }

    public static Limelight createLimelight(HardwareMap hardwareMap) {
        assertHasMechanism(Limelight.class);
        return currentBot.createMechanism(hardwareMap, Limelight.class);
    }

    public static Drivetrain createDrivetrain(HardwareMap hardwareMap) {
        assertHasMechanism(Drivetrain.class);
        return currentBot.createMechanism(hardwareMap, Drivetrain.class);
    }

    public static RGBIndicator createRGB(HardwareMap hardwareMap) {
        assertHasMechanism(RGBIndicator.class);
        return currentBot.createMechanism(hardwareMap, RGBIndicator.class);
    }

    public static LogitechWebcam createWebcam(HardwareMap hardwareMap) {
        assertHasMechanism(LogitechWebcam.class);
        return currentBot.createMechanism(hardwareMap, LogitechWebcam.class);
    }

    public static Slides1 createSlides1(HardwareMap hardwareMap) {
        assertHasMechanism(Slides1.class);
        return currentBot.createMechanism(hardwareMap, Slides1.class);
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
