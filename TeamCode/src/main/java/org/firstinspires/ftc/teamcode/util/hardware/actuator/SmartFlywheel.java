package org.firstinspires.ftc.teamcode.util.hardware.actuator;

import androidx.annotation.NonNull;

import com.pedropathing.revhub.drivetrains.CachedMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.util.Direction;
import org.firstinspires.ftc.teamcode.util.control.Controller;

import java.util.function.DoubleUnaryOperator;

public class SmartFlywheel {
    public CachedMotor motor;
    public double targetTicksPerSecond, targetPower;
    public boolean runningToVelocity;
    public Controller<Double, Double, Double> velocityController;
    public Params params;

    public SmartFlywheel(@NonNull DcMotorEx dcMotorEx, @NonNull Controller<Double, Double, Double> velocityController, @NonNull Params params) {
        motor = new CachedMotor(dcMotorEx, 0.005);
        this.velocityController = velocityController;
        this.params = params;
        setUpMotor();
    }

    private void setUpMotor() {
        motor.raw().setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
    }

    public void setPower(double power) {
        targetPower = power;
        runningToVelocity = false;
    }

    public void setVelocity(double velocity) {
        targetTicksPerSecond = velocity;
        runningToVelocity = true;
        velocityController.setTarget(targetTicksPerSecond);
    }

    public void setRPM(double rpm) {
        targetTicksPerSecond = params.convertToTPS(rpm);
        runningToVelocity = true;
        velocityController.setTarget(targetTicksPerSecond);
    }

    public void stopImmediately() {
        motor.setPower(0);
    }

    public void update() {
        if (runningToVelocity) {
            velocityController.update(motor.raw().getVelocity());
            motor.setPower(velocityController.getOutput());
        } else {motor.setPower(targetPower);}
    }

    public Direction getDirection() {
        return motor.raw().getDirection() == com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD
                ? Direction.FORWARD : Direction.REVERSE;
    }

    public double getTicksPerSecond() {return motor.raw().getVelocity();}
    public double getRPM() {return params.convertToRPM(getTicksPerSecond());}

    public static class Params {
        public double encoderCpm, gearRatio;
        public Params(double encoderCpm, double gearRatio) {this.encoderCpm = encoderCpm; this.gearRatio = gearRatio;}
        public double convertToRPM(double ticksPerSec) {return convertTPStoRPM().applyAsDouble(ticksPerSec);}
        public double convertToTPS(double rotationsPerMinute) {return convertRPMtoTPS().applyAsDouble(rotationsPerMinute);}
        public DoubleUnaryOperator convertTPStoRPM() {return tps -> tps * 60/ (encoderCpm*gearRatio);}
        public DoubleUnaryOperator convertRPMtoTPS() {return rpm -> rpm*encoderCpm*gearRatio/60;}
    }
}
