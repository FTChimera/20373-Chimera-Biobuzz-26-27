package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.library.hardware.webcam.Limelight;
import org.firstinspires.ftc.teamcode.library.robot.RobotConfigurations;
import org.firstinspires.ftc.teamcode.library.robot.RobotConstants;
import org.firstinspires.ftc.teamcode.mechanisms.Drivetrain;
import org.firstinspires.ftc.teamcode.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.MecanumDrive;

@Configurable
public class Constants {
    public static final Pose RED_DEFAULT_START_POSE = new Pose(0, 0, 0);
    public static final Pose BLUE_DEFAULT_START_POSE = new Pose(0, 0, 0);
    public static final RobotConstants currentBot = RobotConfigurations.OldDECODERobotTesting();

    public static Follower createFollower(HardwareMap hardwareMap) {
        return currentBot.createFollower(hardwareMap);
    }
    public static MecanumDrive createTeleOpFollower(HardwareMap hardwareMap) {
        return currentBot.createMechanism(hardwareMap, MecanumDrive.class);
    }

    public static Intake createIntake(HardwareMap hardwareMap) {
        return currentBot.createMechanism(hardwareMap, Intake.class);
    }

    public static Limelight createLimelight(HardwareMap hardwareMap) {
        return currentBot.createMechanism(hardwareMap, Limelight.class);
    }

    public static Drivetrain createDrivetrain(HardwareMap hardwareMap) {
        return currentBot.createMechanism(hardwareMap, Drivetrain.class);
    }

}
