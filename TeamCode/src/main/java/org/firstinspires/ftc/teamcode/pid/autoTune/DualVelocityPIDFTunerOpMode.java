package org.firstinspires.ftc.teamcode.pid.autoTune;

import org.firstinspires.ftc.teamcode.pid.autoTune.master.DualMotorPIDMaster;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Auto-tunes PIDF gains for a dual-motor velocity-controlled mechanism
 * (e.g. a two-flywheel shooter) using the relay-feedback method.
 * Both motors are driven with the same output -- the resulting gains
 * are applied identically to both.
 *
 * <p>Set {@link TuningConstants#TARGET_RPM}, {@link TuningConstants#TICKS_PER_REV},
 * both motor names, and directions in {@link TuningConstants}. Everything
 * else runs automatically.
 *
 * <h2>Setup</h2>
 * <ol>
 *     <li>In {@link TuningConstants}, set:
 *         <ul>
 *             <li>{@code MOTOR_NAME} + {@code REVERSED} for motor 1</li>
 *             <li>{@code MOTOR_NAME_2} + {@code REVERSED_2} for motor 2</li>
 *             <li>{@code TARGET_RPM} to your target flywheel speed</li>
 *             <li>{@code TICKS_PER_REV} for your motor</li>
 *             <li>{@code DUAL_ENCODERS = false} if only motor 1 has an encoder</li>
 *         </ul>
 *     </li>
 *     <li>Run this OpMode and press Start.</li>
 *     <li><b>Phase 1:</b> both motors oscillate around TARGET_RPM automatically.</li>
 *     <li><b>Phase 2:</b> feedforward sweep computes kF on both motors.</li>
 *     <li>Read candidate gains from telemetry -- apply identically to both motors.</li>
 *     <li>Hold gamepad1.A to live-test on both motors simultaneously.</li>
 * </ol>
 */
@TeleOp(name = "Dual Velocity PIDF Tuner", group = "FtcAutoTune")
public class DualVelocityPIDFTunerOpMode extends LinearOpMode {

    @Override
    public void runOpMode() {
        TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        
        // Convert RPM target to ticks/sec using: ticks/sec = (RPM * TICKS_PER_REV) / 60.0
        double targetTicksPerSec = TuningConstants.targetTicksPerSec();

        DualMotorPIDMaster pid = new DualMotorPIDMaster(
                hardwareMap,
                TuningConstants.MOTOR_NAME,   TuningConstants.REVERSED,
                TuningConstants.MOTOR_NAME_2, TuningConstants.REVERSED_2,
                targetTicksPerSec,
                TuningConstants.hysteresisTicksPerSec(),
                TuningConstants.RELAY_AMPLITUDE,
                TuningConstants.CYCLES_TO_COLLECT,
                TuningConstants.CYCLES_TO_IGNORE,
                TuningConstants.RELAY_TEST_TIMEOUT_S,
                TuningConstants.FEEDFORWARD_TEST_POWERS,
                TuningConstants.FEEDFORWARD_SETTLE_TIME_S,
                TuningConstants.TUNE_INTEGRAL_TERM,
                TuningConstants.DUAL_ENCODERS,
                TuningConstants.VELOCITY_LIVE_TEST_GAIN);

        panelsTelemetry.addLine("=== PIDF Auto Tuner (Dual Velocity) ===");
        panelsTelemetry.addData("Motor 1", TuningConstants.MOTOR_NAME + (TuningConstants.REVERSED ? " (reversed)" : ""));
        panelsTelemetry.addData("Motor 2", TuningConstants.MOTOR_NAME_2 + (TuningConstants.REVERSED_2 ? " (reversed)" : ""));
        panelsTelemetry.addData("Encoder mode", TuningConstants.DUAL_ENCODERS ? "DUAL (both motors)" : "SINGLE (motor 1 only)");
        panelsTelemetry.addData("TICKS_PER_REV", TuningConstants.TICKS_PER_REV);
        panelsTelemetry.addData("Target", String.format("%.1f RPM  (%.1f ticks/s)",
                TuningConstants.TARGET_RPM, targetTicksPerSec));
        panelsTelemetry.addLine("Press START. Both motors will oscillate automatically.");
        panelsTelemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive() && !pid.isTuningComplete()) {
            pid.tuningStep(getRuntime());
            for (String line : pid.getTelemetryLines()) panelsTelemetry.addLine(line);
            panelsTelemetry.update();
        }

        if (pid.timedOut() || !pid.isTuningSuccessful()) {
            while (opModeIsActive()) {
                for (String line : pid.getTelemetryLines()) panelsTelemetry.addLine(line);
                panelsTelemetry.update();
            }
            pid.stop();
            return;
        }

        while (opModeIsActive()) {
            for (String line : pid.getResultTelemetryLines()) panelsTelemetry.addLine(line);
            if (gamepad1.a) {
                panelsTelemetry.addLine("");
                for (String line : pid.liveTestStep(getRuntime())) panelsTelemetry.addLine(line);
            } else {
                pid.stopLiveTest();
            }
            panelsTelemetry.update();
        }

        pid.stop();
    }
}
