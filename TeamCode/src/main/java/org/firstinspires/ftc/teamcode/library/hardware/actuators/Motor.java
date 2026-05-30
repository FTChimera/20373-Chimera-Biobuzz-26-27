package org.firstinspires.ftc.teamcode.library.hardware.actuators;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

public class Motor {
    double targetPower, targetVelocity;
    DcMotorEx motor;

    public Motor(HardwareMap hwMap, String name) {
        motor = hwMap.get(DcMotorEx.class, name);
    }
    public Motor reverse() {
        reverseDirection();
        return this;
    }
    public void reverseDirection() {
        motor.setDirection(motor.getDirection().inverted());
    }

    public Motor Float() {
        setFloat();
        return this;
    }

    public void setFloat() {
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    public Motor Brake() {
        setBrake();
        return this;
    }

    public void setBrake() {
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public Motor PIDFControl(PIDFCoefficients coefficients) {
        setPIDFCoefficients(coefficients);
        return this;
    }
    public Motor PIDFControl(double P, double I, double D, double F) {
        return PIDFControl(new PIDFCoefficients(P, I, D, F));
    }

    public void setPIDFCoefficients(double P, double I, double D, double F) {
        setPIDFCoefficients(new PIDFCoefficients(P, I, D, F));
    }
    public void setPIDFCoefficients(PIDFCoefficients coefficients) {
        motor.setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER,
                coefficients
        );
    }

    public DcMotorEx getMotor() {
        return motor;
    }


    public void setPower(double power) {
        targetPower = power;
        motor.setPower(targetPower);
    }

    public double getPower() {
        return motor.getPower();
    }
    public double getTargetPower() {
        return targetPower;
    }
    public double getTargetVelocity() {
        return targetVelocity;
    }
    public void stopVelocity() {
        motor.setVelocity(0);
        targetVelocity = 0;
    }
    public void setVelocity(double velocity) {
        targetVelocity = velocity;
        motor.setVelocity(targetVelocity);
    }
    public void stopPower() {
        motor.setPower(0);
        targetPower = 0;
    }

    public Motor setDirection(Direction motorDirection) {
        if (motor.getDirection() != motorDirection.toFtcSdk()) {
            reverseDirection();
        }
        return this;
    }
    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        motor.setZeroPowerBehavior(zeroPowerBehavior);
    }
}
