package org.firstinspires.ftc.teamcode.mechanisms;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.library.game.State;
import org.firstinspires.ftc.teamcode.library.game.RobotMode;
import org.firstinspires.ftc.teamcode.library.hardware.actuators.Direction;
import org.firstinspires.ftc.teamcode.library.hardware.actuators.Motor;
import org.firstinspires.ftc.teamcode.library.hardware.gamepad.GpCommand;
import org.firstinspires.ftc.teamcode.library.hardware.gamepad.Trigger;
import org.firstinspires.ftc.teamcode.library.robot.RobotConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.CommandConverter;

import java.util.function.Supplier;

public class Intake extends Mechanism {
    public static class INTAKE_PARAMS implements Mechanism.Params<INTAKE_PARAMS> {
        public double FULL_POWER = 1.0;
        public double HALF_POWER = 0.5;
        public double REVERSE_FULL_POWER = -1.0;
        public double STOP_POWER = 0.0;
        public String HARDWAREMAP_NAME = "intake";
        public Direction INTAKE_MOTOR_DIRECTION = Direction.REVERSE;
        public GpCommand FORWARD_COMMAND = Trigger.RIGHT;
        public GpCommand REVERSE_COMMAND = Trigger.LEFT;
        public boolean ISBEINGUSED = true;
        public INTAKE_PARAMS() {}
        private INTAKE_PARAMS(INTAKE_PARAMS other) {
            FULL_POWER = other.FULL_POWER;
            HALF_POWER = other.HALF_POWER;
            REVERSE_FULL_POWER = other.REVERSE_FULL_POWER;
            STOP_POWER = other.STOP_POWER;
            HARDWAREMAP_NAME = other.HARDWAREMAP_NAME;
            INTAKE_MOTOR_DIRECTION = other.INTAKE_MOTOR_DIRECTION;
            FORWARD_COMMAND = other.FORWARD_COMMAND;
            REVERSE_COMMAND = other.REVERSE_COMMAND;
            ISBEINGUSED = other.ISBEINGUSED;
        }
        public INTAKE_PARAMS fullPower(double fullPower) {
            this.FULL_POWER = fullPower;
            return this;
        }
        public INTAKE_PARAMS halfPower(double halfPower) {
            this.HALF_POWER = halfPower;
            return this;
        }
        public INTAKE_PARAMS reverseFullPower(double reverseFullPower) {
            this.REVERSE_FULL_POWER = reverseFullPower;
            return this;
        }
        public INTAKE_PARAMS stopPower(double stopPower) {
            this.STOP_POWER = stopPower;
            return this;
        }
        public INTAKE_PARAMS hardwareMapName(String hardwareMapName) {
            this.HARDWAREMAP_NAME = hardwareMapName;
            return this;
        }
        public INTAKE_PARAMS intakeMotorDirection(Direction intakeMotorDirection) {
            this.INTAKE_MOTOR_DIRECTION = intakeMotorDirection;
            return this;
        }
        public INTAKE_PARAMS forwardCommand(GpCommand forwardCommand) {
            this.FORWARD_COMMAND = forwardCommand == null ? Trigger.RIGHT : forwardCommand;
            return this;
        }
        public INTAKE_PARAMS reverseCommand(GpCommand reverseCommand) {
            this.REVERSE_COMMAND = reverseCommand == null ? Trigger.LEFT : reverseCommand;
            return this;
        }
        public INTAKE_PARAMS isBeingUsed(boolean isBeingUsed) {
            this.ISBEINGUSED = isBeingUsed;
            return this;
        }
        @Override
        public INTAKE_PARAMS updateFromBot(RobotConstants bot) {
            if (!ISBEINGUSED) return this;
            HARDWAREMAP_NAME = bot.Mechanisms.contains(Intake.class) ? HARDWAREMAP_NAME : "";
            ISBEINGUSED = bot.Mechanisms.contains(Intake.class);
            return this;
        }
        @Override
        public INTAKE_PARAMS copy() {
            return new INTAKE_PARAMS(this);
        }
    }
    public Motor Intake_Motor;
    public INTAKE_PARAMS params;
    public Intake(HardwareMap map, RobotConstants bot) {
        super(map, bot);
        this.params = resolveParams(Intake.class, new INTAKE_PARAMS());
        if (!params.ISBEINGUSED) {
            return;
        }
        this.Intake_Motor = new Motor(map, params.HARDWAREMAP_NAME);
        Intake_Motor.setDirection(params.INTAKE_MOTOR_DIRECTION);
        Stop();
    }

    @Override
    public void stop() {
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
     public Command stopCmd() {
         return setPowerCommand(() -> params.STOP_POWER);
     }
        /** set custom power as an ivy command*/
     public Command setPowerCommand(Supplier<Double> power) {
         if (!params.ISBEINGUSED) return CommandConverter.toCommand(() -> {});
         return CommandConverter.toCommand(() -> this.setPower(power.get()));
     }

    @Override
    public void periodic(State state, RobotMode robotMode) {
        if (!params.ISBEINGUSED) {
            return;
        }
        if (robotMode == RobotMode.AUTONOMOUS) {
            return;
        }
        if (state == null) {
            Stop();
            return;
        }
        if (state.getGpBindings() == null) {
            return;
        }
        GpCommand forwardCommand = state.gpBindings.getCommand(params.FORWARD_COMMAND);
        GpCommand reverseCommand = state.gpBindings.getCommand(params.REVERSE_COMMAND);
        double power = params.STOP_POWER;
        if (forwardCommand != null) {
            power += forwardCommand.getValue() * params.FULL_POWER;
        }
        if (reverseCommand != null) {
            power += reverseCommand.getValue() * params.REVERSE_FULL_POWER;
        }
        setPower(power);
    }

}
