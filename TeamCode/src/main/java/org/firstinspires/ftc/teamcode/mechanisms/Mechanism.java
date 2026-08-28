package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.library.game.RobotMode;
import org.firstinspires.ftc.teamcode.library.game.State;
import org.firstinspires.ftc.teamcode.library.robot.RobotConstants;

public abstract class Mechanism {
    public interface Params<T extends Params<T>> {
        T copy();
        T updateFromBot(RobotConstants bot);
    }

    protected HardwareMap hardwareMap;
    protected RobotConstants bot;
    public Mechanism(HardwareMap map, RobotConstants bot) {
        hardwareMap = map;
        this.bot = bot;
    }

    protected <P extends Params<P>> P resolveParams(Class<? extends Mechanism> mechanismClass, P defaultParams) {
        return bot.getMechanismParams(mechanismClass, defaultParams).updateFromBot(bot);
    }

    public void periodic(State robotState) {
        periodic(robotState, robotState == null ? RobotMode.TELEOP : robotState.getRobotMode());
    }

    public abstract void periodic(State robotState, RobotMode robotMode);
}
