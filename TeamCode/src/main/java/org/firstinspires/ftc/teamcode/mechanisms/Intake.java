package org.firstinspires.ftc.teamcode.mechanisms;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.library.hardware.actuators.Direction;
import org.firstinspires.ftc.teamcode.library.hardware.actuators.Motor;
import org.firstinspires.ftc.teamcode.library.hardware.hardwareDevice.SlidesMotor;
import org.firstinspires.ftc.teamcode.library.hardware.gamepad.Bindings;
import org.firstinspires.ftc.teamcode.library.robot.RobotConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.CommandConverter;

import java.util.function.Supplier;

public class Intake extends Mechanism {
    public static class INTAKE_PARAMS implements Mechanism.Params<INTAKE_PARAMS> {
        public double FULL_POWER = 1.0;
        public double HALF_POWER = 0.5;
        public double REVERSE_FULL_POWER = -1.0;
        public double STOP_POWER = 0.0;
        public int SLIDES_MAX = 1400;
        public int SLIDES_MIN = 0;
        public String HARDWAREMAP_NAME_SLIDES = "intakeSlides";
        public String HARDWAREMAP_NAME = "intake";
        public Direction SLIDES_MOTOR_DIRECTION = Direction.REVERSE;
        public Direction INTAKE_MOTOR_DIRECTION = Direction.REVERSE;
        public boolean ISBEINGUSED = true;
        public INTAKE_PARAMS() {}
        private INTAKE_PARAMS(INTAKE_PARAMS other) {
            FULL_POWER = other.FULL_POWER;
            HALF_POWER = other.HALF_POWER;
            REVERSE_FULL_POWER = other.REVERSE_FULL_POWER;
            STOP_POWER = other.STOP_POWER;
            SLIDES_MAX = other.SLIDES_MAX;
            SLIDES_MIN = other.SLIDES_MIN;
            HARDWAREMAP_NAME_SLIDES = other.HARDWAREMAP_NAME_SLIDES;
            HARDWAREMAP_NAME = other.HARDWAREMAP_NAME;
            SLIDES_MOTOR_DIRECTION = other.SLIDES_MOTOR_DIRECTION;
            INTAKE_MOTOR_DIRECTION = other.INTAKE_MOTOR_DIRECTION;
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
        public INTAKE_PARAMS slidesHardwareMapName(String hardwareMapNameSlides) {
            this.HARDWAREMAP_NAME_SLIDES = hardwareMapNameSlides;
            return this;
        }
        public INTAKE_PARAMS slidesMin(int slidesMin) {
            this.SLIDES_MIN = slidesMin;
            return this;
        }
        public INTAKE_PARAMS slidesMax(int slidesMax) {
            this.SLIDES_MAX = slidesMax;
            return this;
        }
        public INTAKE_PARAMS intakeMotorDirection(Direction intakeMotorDirection) {
            this.INTAKE_MOTOR_DIRECTION = intakeMotorDirection;
            return this;
        }
        public INTAKE_PARAMS slidesMotorDirection(Direction slidesMotorDirection) {
            this.SLIDES_MOTOR_DIRECTION = slidesMotorDirection;
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
    public SlidesMotor slidesMotor;
    public INTAKE_PARAMS params;
    public Intake(HardwareMap map, RobotConstants bot) {
        super(map, bot);
        this.params = resolveParams(Intake.class, new INTAKE_PARAMS());
        if (!params.ISBEINGUSED) {
            return;
        }
        this.Intake_Motor = new Motor(map, params.HARDWAREMAP_NAME);
        Intake_Motor.setDirection(params.INTAKE_MOTOR_DIRECTION);
        this.slidesMotor = map.get(SlidesMotor.class, params.HARDWAREMAP_NAME_SLIDES);
        slidesMotor.initialize(params.SLIDES_MIN, params.SLIDES_MAX);
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

    @Override
    public void periodic(Bindings bindings) {}

}
