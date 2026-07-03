package org.firstinspires.ftc.teamcode.pid.autoTune;
import com.aaravdhawan25.pidautotuner.ftc.PIDMaster;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Auto-tunes PIDF gains for a velocity-controlled mechanism (flywheel,
 * intake, etc.) using the relay-feedback method.
 *
 * <p>Set {@link TuningConstants#TARGET_RPM} to your target speed in RPM and
 * {@link TuningConstants#TICKS_PER_REV} to your motor's ticks per revolution.
 * Everything else runs automatically.
 *
 * <h2>Setup</h2>
 * <ol>
 *     <li>Set {@code MOTOR_NAME}, {@code TARGET_RPM}, and {@code TICKS_PER_REV}
 *         in {@link TuningConstants}. Verify {@code REVERSED} is correct.</li>
 *     <li>Run this OpMode and press Start.</li>
 *     <li><b>Phase 1 (relay test):</b> motor oscillates around TARGET_RPM automatically.</li>
 *     <li><b>Phase 2 (feedforward sweep):</b> motor ramps through power levels to compute kF.</li>
 *     <li>Read the candidate gain sets from panelsTelemetry.</li>
 *     <li>Hold gamepad1.A to live-test the classic ZN candidate.</li>
 * </ol>
 */
@TeleOp(name = "Velocity PIDF Tuner", group = "FtcAutoTune")
public class VelocityPIDFTunerOpMode extends LinearOpMode {

    @Override
    public void runOpMode() {
        TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        // Convert RPM target to ticks/sec using: ticks/sec = (RPM * TICKS_PER_REV) / 60.0
        double targetTicksPerSec = TuningConstants.targetTicksPerSec();

        PIDMaster pid = new PIDMaster(
                hardwareMap, TuningConstants.MOTOR_NAME, TuningConstants.REVERSED, false,
                targetTicksPerSec, TuningConstants.hysteresisTicksPerSec(),
                TuningConstants.RELAY_AMPLITUDE, TuningConstants.CYCLES_TO_COLLECT, TuningConstants.CYCLES_TO_IGNORE,
                TuningConstants.RELAY_TEST_TIMEOUT_S, TuningConstants.FEEDFORWARD_TEST_POWERS,
                TuningConstants.FEEDFORWARD_SETTLE_TIME_S, TuningConstants.TUNE_INTEGRAL_TERM,
                TuningConstants.TICKS_PER_REV);

        panelsTelemetry.addLine("=== PIDF Auto Tuner (Velocity) ===");
        panelsTelemetry.addData("Motor", TuningConstants.MOTOR_NAME);
        panelsTelemetry.addData("TICKS_PER_REV", TuningConstants.TICKS_PER_REV);
        panelsTelemetry.addData("Target", String.format("%.1f RPM  (%.1f ticks/s)",
                TuningConstants.TARGET_RPM, targetTicksPerSec));
        panelsTelemetry.addLine("Press START. Phase 1: relay test (automatic).");
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
                panelsTelemetry.update();
                for (String line : pid.getTelemetryLines()) panelsTelemetry.addLine(line);
                panelsTelemetry.update();
            }
            return;
        }

        while (opModeIsActive()) {
            panelsTelemetry.update();
            for (String line : pid.getResultTelemetryLines()) panelsTelemetry.addLine(line);
            if (gamepad1.a) {
                panelsTelemetry.addLine("");
                for (String line : pid.liveTestStep(getRuntime())) panelsTelemetry.addLine(line);
            } else {
                pid.stopLiveTest();
            }
            panelsTelemetry.update();
        }
    }
}