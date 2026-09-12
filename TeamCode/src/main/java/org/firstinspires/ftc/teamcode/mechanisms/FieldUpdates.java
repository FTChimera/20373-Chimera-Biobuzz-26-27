package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.library.game.RobotMode;
import org.firstinspires.ftc.teamcode.library.game.State;
import org.firstinspires.ftc.teamcode.library.robot.RobotConstants;

/** The purpose of this class is to run updates on the field like hive positions, as a mechanism (periodic every loop) */
public class FieldUpdates extends Mechanism {
    public FieldUpdates(HardwareMap map, RobotConstants bot) {
        super(map, bot);
    }

    @Override
    public void stop() {

    }

    @Override
    public void periodic(State robotState, RobotMode robotMode) {

    }
}
