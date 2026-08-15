package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.HardwareMap;

import com.rmsftc.library.hardware.gamepad.GpBindings;
import com.rmsftc.library.robot.RobotConstants;

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

    public abstract void periodic(GpBindings GpBindings);
}
