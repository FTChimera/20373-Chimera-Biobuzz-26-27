package org.firstinspires.ftc.teamcode.library.control;

public interface Controller<StateT, TargetT, OutputT> {
    void update(StateT state);
    void reset();
    void setTarget(TargetT target);
    TargetT getTarget();
    OutputT getOutput();
    boolean atTarget();

    default OutputT calculate(StateT state) {
        update(state);
        return getOutput();
    }
}
