package org.firstinspires.ftc.teamcode.mechanisms;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.library.game.Field;
import org.firstinspires.ftc.teamcode.library.game.HiveState;
import org.firstinspires.ftc.teamcode.library.game.State;
import org.firstinspires.ftc.teamcode.library.robot.RobotConstants;

/** The purpose of this class is to run updates on the field like hive positions, as a mechanism (periodic every loop) */
public class FieldUpdates extends Mechanism {
    HiveState hiveState = HiveState.UNTIPPED;

    public FieldUpdates(HardwareMap map, RobotConstants bot) {
        super(map, bot);
    }

    @Override
    public void stop() {

    }

    @Override
    public void periodic(State robotState) {

    }

    public void setHiveState(HiveState hiveState) {
        this.hiveState = hiveState;
    }
    public Pose getGoalPose() {
        return null;
    }
}
