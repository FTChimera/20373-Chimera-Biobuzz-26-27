package org.firstinspires.ftc.teamcode.mechanisms;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.library.hardware.actuators.Motor;
import org.firstinspires.ftc.teamcode.pedroPathing.CommandConverter;

import java.util.function.Supplier;

public class Intake {
    public static class INTAKE_PARAMS {
        public final double FULL_POWER = 1.0;
        public final double HALF_POWER = 0.5;
        public final double REVERSE_FULL_POWER = -1.0;
        public final double STOP_POWER = 0.0;
        public final String HARDWAREMAP_NAME = "intake";
        public final DcMotorSimple.Direction INTAKE_MOTOR_DIRECTION = DcMotorSimple.Direction.REVERSE;
    }
    public Motor Intake_Motor;
    public INTAKE_PARAMS params;
    public Intake(HardwareMap map, INTAKE_PARAMS params) {
        this.Intake_Motor = new Motor(map, params.HARDWAREMAP_NAME);
        Intake_Motor.setDirection(params.INTAKE_MOTOR_DIRECTION);
        this.params = params;
        Stop();
    }

    public void setPower(double power) {
        Intake_Motor.setPower(power);
    }
    public void FullPower() {
        setPower(params.FULL_POWER);
    }
    public void HalfPower() {
        setPower(params.HALF_POWER);
    }
    public void ReverseFullPower() {
        setPower(params.REVERSE_FULL_POWER);
    }
    public void Stop() {
        setPower(params.STOP_POWER);
    }

    public double getTargetPower() {
        return Intake_Motor.getTargetPower();
    }

    // Ivy methods
    /** Set full power as an Ivy Command*/
    public Command fullPower() {
        return setPowerCommand(() -> params.FULL_POWER);
    }
    /** set reverse power as an ivy command*/
    public Command reverseFullPower() {
        return setPowerCommand(() -> params.REVERSE_FULL_POWER);
    }
    /** set half power as an ivy command*/
    public Command halfPower() {
        return setPowerCommand(() -> params.HALF_POWER);
    }
    /** set stop as an ivy command*/
     public Command stop() {
         return setPowerCommand(() -> params.STOP_POWER);
     }
        /** set custom power as an ivy command*/
     public Command setPowerCommand(Supplier<Double> power) {
         return CommandConverter.toCommand(() -> this.setPower(power.get()));
     }

}
