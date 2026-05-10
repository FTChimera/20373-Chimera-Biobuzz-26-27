package org.firstinspires.ftc.teamcode.mechanisms;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.library.hardware.actuators.Motor;
import org.firstinspires.ftc.teamcode.pedroPathing.CommandConverter;

import java.util.function.Supplier;

public class Intake {
    public Motor Intake_Motor;
    public Intake(HardwareMap map) {
        this.Intake_Motor = new Motor(map, Constants.INTAKE_MOTOR_NAME);
        Intake_Motor.setDirection(Constants.INTAKE_MOTOR_DIRECTION);
        Stop();
    }

    public void setPower(double power) {
        Intake_Motor.setPower(power);
    }
    public void FullPower() {
        setPower(1);
    }
    public void HalfPower() {
        setPower(0.5);
    }
    public void ReverseFullPower() {
        setPower(-1);
    }
    public void Stop() {
        setPower(0);
    }

    public double getTargetPower() {
        return Intake_Motor.getTargetPower();
    }

    // Ivy methods
    /** Set full power as an Ivy Command*/
    public Command fullPower() {
        return setPowerCommand(() -> 1.0);
    }
    /** set reverse power as an ivy command*/
    public Command reverseFullPower() {
        return setPowerCommand(() -> -1.0);
    }
    /** set half power as an ivy command*/
    public Command halfPower() {
        return setPowerCommand(() -> 0.5);
    }
    /** set stop as an ivy command*/
     public Command stop() {
         return setPowerCommand(() -> 0.0);
     }
        /** set custom power as an ivy command*/
     public Command setPowerCommand(Supplier<Double> power) {
         return CommandConverter.toCommand(() -> this.setPower(power.get()));
     }

}
