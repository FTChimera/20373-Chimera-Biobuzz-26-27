package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.library.robot.RobotConstants;

public abstract class Mechanism {
    // inheritors
    // drivetrain, intake, mecanumdrive, rgb indicator, limelight, logitechwebcam
    protected HardwareMap hardwareMap;
    protected RobotConstants bot;
    public Mechanism(HardwareMap map, RobotConstants bot) {
        hardwareMap = map;
        this.bot = bot;
    }
}
