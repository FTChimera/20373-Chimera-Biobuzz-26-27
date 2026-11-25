package org.firstinspires.ftc.teamcode.util.control;

import com.rmsftc.utils.Clock;

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

    static Controller<Double, Double, Double> doubleFlywheelFullVelocity(DoubleFlywheelFullVelocityConstants constants) {
        return new DoubleFlywheelFullVelocityController(constants);
    }

    static Controller<Double, Double, Double> doubleFlywheelFullVelocity(DoubleFlywheelFullVelocityConstants constants, Clock clock) {
        return new DoubleFlywheelFullVelocityController(constants, clock);
    }

    static DoublePIDFController doublePIDF(DoublePIDFCoeffs coeffs) {
        return new DoublePIDFController(coeffs);
    }
}
