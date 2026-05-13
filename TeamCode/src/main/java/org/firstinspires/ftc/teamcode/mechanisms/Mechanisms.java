package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.library.hardware.webcam.Limelight;

public class Mechanisms {
    public static Drivetrain drivetrain(HardwareMap map) {
        return new Drivetrain(map);
    }

    public static Limelight limelight(HardwareMap map) {
        return new Limelight(map, Constants.LIMELIGHT_CAMERA_NAME);
    }

    public static Intake intake(HardwareMap map) {
        return new Intake(map);
    }

}
