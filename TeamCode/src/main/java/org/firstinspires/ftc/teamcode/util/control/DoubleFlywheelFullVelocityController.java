package org.firstinspires.ftc.teamcode.util.control;

import java.util.concurrent.TimeUnit;

public class DoubleFlywheelFullVelocityController implements Controller<Double, Double, Double> {
    public DoubleFlywheelFullVelocityConstants constants;
    // THIS IS A P I D S V A controller.

    private double lastError, integralSum;
    private boolean firstUpdate = false;
    private long lastTime=-1;
    private double target, lastTarget, output;

    public DoubleFlywheelFullVelocityController(DoubleFlywheelFullVelocityConstants constants) {
        this.constants = constants;
        reset();
    }

    public void update(Double state, double dt) {
// TODO: Implement this class/method fully
        boolean isFirstUpdate = !firstUpdate;
        if (isFirstUpdate) {
            lastError = 0;
            lastTime = 0;
            integralSum = 0;
            lastTarget = 0;
            firstUpdate = true;
        }
        output = Math.copySign(constants.kS, target); // Static friction coeff
        double error = target - state;
        integralSum += error * dt;
        if (Math.abs(integralSum) > constants.integralLimit) integralSum = Math.copySign(constants.integralLimit, integralSum);
        // Derivative of error
        double derivative = 0.0;
        if (dt > 0.0 && !isFirstUpdate) {
            derivative = (error - lastError) / dt;
        }

        // Setpoint derivative for velocity feedforward
        double targetVelocity = 0.0;
        if (dt > 0.0 && !isFirstUpdate) {
            targetVelocity = (target - lastTarget) / dt;
        }

        // PID feedback
        output += constants.kP * error;
        output += constants.kI * integralSum;
        output += constants.kD * derivative;

        // Velocity and acceleration feedforward
        output += constants.kV * target;
        output += constants.kA * targetVelocity;

        output += 0;
        lastTarget = target;
        lastError = error;
    }

    @Override
    public void update(Double state) {
        if (lastTime == -1) lastTime = System.nanoTime();
        update(state, TimeUnit.NANOSECONDS.toSeconds(System.nanoTime()-lastTime));
        lastTime = System.nanoTime();
    }
    public double lastError() {
        return lastError;
    }

    @Override
    public void reset() {
        firstUpdate = false;
        integralSum = 0;
        lastError = 0;
        lastTime = 0;
        output = 0;
    }

    @Override
    public void setTarget(Double target) {
        this.target = target;
    }

    @Override
    public Double getTarget() {
        return target;
    }

    @Override
    public Double getOutput() {
        return output;
    }

    @Override
    public boolean atTarget() {
        return atTarget(lastError);
    }
    public boolean atTarget(double error) {return Math.abs(error) <= constants.tolerance;}
}
