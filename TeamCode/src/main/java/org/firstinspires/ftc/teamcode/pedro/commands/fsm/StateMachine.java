package org.firstinspires.ftc.teamcode.pedro.commands.fsm;

import org.firstinspires.ftc.teamcode.util.Timer;

import java.util.concurrent.TimeUnit;
import java.util.function.Supplier;

public abstract class StateMachine implements BasicStateMachine {
    protected int state = 0;
    protected Timer pathTimer;
    public static StateMachine create(Supplier<Boolean> runFunction) {
        return new StateMachine() {
            @Override
            public boolean run() {
                return runFunction.get();
            }
        };
    }

    public static StateMachine from(BasicStateMachine basicStateMachine) {
        return create(basicStateMachine::run);
    }

    public void setState(int newPathState) {
        state = newPathState;
        pathTimer = new Timer(TimeUnit.SECONDS);
    }

    public void setState(Enum<?> constant) {
        setState(constant.ordinal());
    }

    @Override
    public abstract boolean run();

}
