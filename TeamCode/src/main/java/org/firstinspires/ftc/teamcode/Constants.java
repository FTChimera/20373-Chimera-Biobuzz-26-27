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

@Configurable
public class Constants {
    // Params
    public static final Intake.INTAKE_PARAMS INTAKE_PARAMS = new Intake.INTAKE_PARAMS();
    public static final Limelight.LIMELIGHT_PARAMS LIMELIGHT_PARAMS = new Limelight.LIMELIGHT_PARAMS();
    public static final Pose RED_DEFAULT_START_POSE = new Pose(0, 0, 0);
    public static final Pose BLUE_DEFAULT_START_POSE = new Pose(0, 0, 0);
    public static final boolean DRIVE_FIELD_RELATIVE = false;

    public static final RobotConstants currentBot = RobotConfigurations.starterBot();

    public static Follower createFollower(HardwareMap hardwareMap) {
        return currentBot.createFollower(hardwareMap);
    }

    public static Intake createIntake(HardwareMap hardwareMap) {
        return new Intake(hardwareMap, INTAKE_PARAMS, currentBot);
    }

    public static Limelight createLimelight(HardwareMap hardwareMap) {
        return new Limelight(hardwareMap, LIMELIGHT_PARAMS, currentBot);
    }

    public static Drivetrain createDrivetrain(HardwareMap hardwareMap) {
        return new Drivetrain(hardwareMap, currentBot);
    }

}
