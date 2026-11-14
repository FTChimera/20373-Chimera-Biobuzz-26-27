package org.firstinspires.ftc.teamcode.pedro.procedures;

import com.aaravdhawan25.pidautotuner.RelayTuningResult;
import com.aaravdhawan25.pidautotuner.ftc.PIDMaster;
import com.pedropathing.tuning.autotune.Inputs;
import com.pedropathing.tuning.autotune.Procedure;
import com.pedropathing.tuning.autotune.TuningOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.util.Tuple;

import java.util.function.Function;


public class MotorPIDVelocityTuner extends Procedure {
    public enum ControlType {
        P,
        PD_GENTLE,
        PD_NO_OVERSHOOT,
        PD_SOME_OVERSHOOT,
        PD_CLASSIC_ZN,
        PD_AGGRESSIVE,
        PI,
        PID_NO_OVERSHOOT,
        PID_SOME_OVERSHOOT,
        PID_CLASSIC_ZN,
        PID_PESSEN_AGGRESSIVE;
        public boolean integralTerm() {
            return this.name().startsWith("PI");
        }
    }

    public MotorPIDVelocityTuner() {
        super("Motor PID Velocity Tuner", "Tunes the control gains/values for a motor running velocity mode");
    }

    @Override
    public void run() throws InterruptedException {
        Inputs inputs = inputs("Motor config", "Enter the motor configuration and control type for tuning");
        Inputs.Field<String> name = inputs.s("Motor HardwareMap name");
        Inputs.Field<Direction> direction = inputs.e("Motor Direction", Direction.class);
        Inputs.Field<ControlType> controlType = inputs.e("Control Type", ControlType.class);

        awaitInputs(inputs);

        confirmation("PID Auto Tuner", "The flywheel will spin and automatically tune PIDF values (based on the control type you chose). You will record those values to use in a PIDF Controller. You selected this: \n" + controlType.get());
        confirmation("Assumptions", "This tuner assumes that you have a velocity-controlled (flywheel, hopefully) motor able to run freely. It also assumes that you have a single-motor setup. If any of those conditions are not met, modify the code/hardware to meet them, or this is simply not applicable");

        RelayTuningResult result = runOpMode(new MotorPIDAutoTunerOpMode(name.get(), direction.get()));
double kF = 0;
        // TODO: Complete Motor PID Velocity Tuner
        // now for the zeiger nichols stuff
        ControlType controlType1 = controlType.get();
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients();
        double Ku = result.Ku;
        double Tu = result.Tu;
        switch (controlType1) {
            case P:
                pidfCoefficients = new PIDFCoefficients(0.5*result.Ku, 0, 0, kF);
                break;
            case PD_GENTLE:
                pidfCoefficients = new PIDFCoefficients(0.3*Ku, 0, 0.3*Ku*Tu/4, kF);
                break;
            case PD_NO_OVERSHOOT:
                // Ziegler-Nichols "no overshoot" PD variant (same table as util/autotune/ZieglerNichols).
                pidfCoefficients = new PIDFCoefficients(0.40*Ku, 0, (0.40*Ku)*(Tu/3.0), kF);
                break;
            case PD_SOME_OVERSHOOT:
                pidfCoefficients = new PIDFCoefficients(0.60*Ku, 0, (0.60*Ku)*(Tu/4.0), kF);
                break;
            case PD_CLASSIC_ZN:
                pidfCoefficients = new PIDFCoefficients(0.80*Ku, 0, (0.80*Ku)*(Tu/8.0), kF);
                break;
            case PD_AGGRESSIVE:
                pidfCoefficients = new PIDFCoefficients(1.20*Ku, 0, (1.20*Ku)*(Tu/8.0), kF);
                break;
            case PI:
                // Textbook ZN PI: Kp=0.45 Ku, Ti=Tu/1.2, Ki=Kp/Ti=0.54 Ku/Tu.
                pidfCoefficients = new PIDFCoefficients(0.45*Ku, 0.54*Ku/Tu, 0, kF);
                break;
            case PID_NO_OVERSHOOT:
                pidfCoefficients = new PIDFCoefficients(0.20*Ku, 0.40*Ku/Tu, (0.20*Ku)*(Tu/3.0), kF);
                break;
            case PID_SOME_OVERSHOOT:
                pidfCoefficients = new PIDFCoefficients(0.33*Ku, 0.66*Ku/Tu, (0.33*Ku)*(Tu/3.0), kF);
                break;
            case PID_CLASSIC_ZN:
                pidfCoefficients = new PIDFCoefficients(0.60*Ku, 1.20*Ku/Tu, (0.60*Ku)*(Tu/8.0), kF);
                break;
            case PID_PESSEN_AGGRESSIVE:
                pidfCoefficients = new PIDFCoefficients(0.70*Ku, 1.75*Ku/Tu, (0.70*Ku)*(0.15*Tu), kF);
                break;
            default:
                pidfCoefficients = new PIDFCoefficients();
        }

        result("P", pidfCoefficients.p);
        result("I", pidfCoefficients.i);
        result("D", pidfCoefficients.d);
        result("F", pidfCoefficients.f);
    }
}

class MotorPIDAutoTunerOpMode extends TuningOpMode<RelayTuningResult> {
    Tuple<String, Boolean> motor;
    public MotorPIDAutoTunerOpMode(String name, Direction reversed) {
        super("Motor PID Auto Tuner OpMode", "Tunes the PIDF Values of a specified motor using the FtcAutoTune dependency, and returns the RelayTuningResult", true);
       motor = new Tuple<>(name, reversed == Direction.REVERSE);
    }

    @Override
    protected RelayTuningResult runTuningOpMode() throws InterruptedException {
        PIDMaster pidMaster = new PIDMaster(
                hardwareMap,
                motor.getA(),
                motor.getB(),
                false,
                0,
                0,
                0,
                0,
                0,
                0,
                new double[0],
                0,
                false,
                0
        );
        waitForStart();

        return new RelayTuningResult(0,0,0,0);
    }
}
