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
import org.firstinspires.ftc.teamcode.library.robot.RobotConfigurations;
import org.firstinspires.ftc.teamcode.library.robot.RobotConstants;

@Configurable
public class Constants {
    public static final String INTAKE_MOTOR_NAME = "intake";
    public static final String LIMELIGHT_CAMERA_NAME = "limelight";
    public static final DcMotorSimple.Direction INTAKE_MOTOR_DIRECTION = DcMotorSimple.Direction.REVERSE;
    public static final Pose RED_DEFAULT_START_POSE = new Pose(0, 0, 0);
    public static final Pose BLUE_DEFAULT_START_POSE = new Pose(0, 0, 0);
    public static final boolean DRIVE_FIELD_RELATIVE = false;

    public static final RobotConstants currentBot = RobotConfigurations.starterBot();

    public static Follower createFollower(HardwareMap hardwareMap) {
        return currentBot.createFollower(hardwareMap);
    }
}
