package org.firstinspires.ftc.teamcode.mechanisms;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.library.hardware.actuators.Direction;
import org.firstinspires.ftc.teamcode.library.hardware.actuators.Motor;
import org.firstinspires.ftc.teamcode.library.robot.RobotConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.CommandConverter;

import java.util.function.Supplier;

public class Intake {
    public static class INTAKE_PARAMS {
        public double FULL_POWER = 1.0;
        public double HALF_POWER = 0.5;
        public double REVERSE_FULL_POWER = -1.0;
        public double STOP_POWER = 0.0;
        public String HARDWAREMAP_NAME = "intake";
        public Direction INTAKE_MOTOR_DIRECTION = Direction.REVERSE;
        public boolean ISBEINGUSED = true;
        public void updateFromBot(RobotConstants bot) {
            if (!ISBEINGUSED) return;
            HARDWAREMAP_NAME = bot.USE_INTAKE ? HARDWAREMAP_NAME : "";
            ISBEINGUSED = bot.USE_INTAKE;
        }
    }
    public Motor Intake_Motor;
    public INTAKE_PARAMS params;
    public Intake(HardwareMap map, INTAKE_PARAMS params, RobotConstants bot) {
        if (!params.ISBEINGUSED) {
            this.params = params;
            this.params.updateFromBot(bot);
            return;
        }
        this.Intake_Motor = new Motor(map, params.HARDWAREMAP_NAME);
        Intake_Motor.setDirection(params.INTAKE_MOTOR_DIRECTION);
        this.params = params;
        this.params.updateFromBot(bot);
        Stop();
    }

    public void setPower(double power) {
        if (!params.ISBEINGUSED) return;
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
        if (!params.ISBEINGUSED) return 0;
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
         if (!params.ISBEINGUSED) return CommandConverter.toCommand(() -> {});
         return CommandConverter.toCommand(() -> this.setPower(power.get()));
     }

}
