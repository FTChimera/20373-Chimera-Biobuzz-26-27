package org.firstinspires.ftc.teamcode.pedro.procedures;

import com.pedropathing.tuning.autotune.Inputs;
import com.pedropathing.tuning.autotune.Procedure;
import com.pedropathing.tuning.autotune.TuningOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.control.DoubleFlywheelFullVelocityConstants;

import java.util.function.Function;

public class ShooterPIDSVATuner extends Procedure {
    public ShooterPIDSVATuner(Function<HardwareMap, DcMotorEx> shooterMotorSupplier, double ticksPerRev) {
        super("Shooter PIDSVA Tuner", "Tunes PIDSVA values for the shooter");
    }

    @Override
    public void run() throws InterruptedException {
        Inputs inputs = inputs("Tuning Inputs", "Inputs for the autotuner");
        Inputs.NumberField<Double> relay_amplitude = inputs.d("Relay Amplitude (bang-bang output magnitude");
        Inputs.NumberField<Integer> cycles_to_collect = inputs.i("Number of Oscillations (Cycles) to Collect");
        Inputs.NumberField<Integer> cycles_to_ignore = inputs.i("Number of Oscillations (Cycles) to Ignore");
        Inputs.NumberField<Double> relay_test_timeout_secs = inputs.d("Relay Test Timeout (seconds)");
        Inputs.NumberField<Integer> target_rpm = inputs.i("Target RPM");
        Inputs.NumberField<Double> velocity_hysteresis_rpm = inputs.d("Velocity Hysteresis (Deadband) (RPM)");
        Inputs.StringField feedforward_test_powers = inputs.s("Feedforward Test Powers (comma-separated)");
        Inputs.NumberField<Double> feedforward_settle_time_secs = inputs.d("Feedforward Settle Time (seconds)");
        Inputs.Field<Boolean> tune_integral_term = inputs.b("Tune Integral Term");

        awaitInputs(inputs);
        confirmation("Flywheel tuning", "Are you ready to start tuning the flywheel?");
        DoubleFlywheelFullVelocityConstants constants = new DoubleFlywheelFullVelocityConstants(0,0,0,0,0,0,0,0);
//        constants.kS = runOpMode(new StaticFrictionCoeffTuner(shooterMotorSupplier));
    }
}
class StaticFrictionCoeffTuner extends TuningOpMode<Double> {

    public StaticFrictionCoeffTuner(Function<HardwareMap, DcMotorEx> shooter) {
        super("Static Friction Coefficient Tuner", "Tunes the static friction coefficient for the shooter", true);
    }
    @Override
    protected Double runTuningOpMode() throws InterruptedException {
        while (!isStopRequested()) {

        }
        return 0.0;
    }
}