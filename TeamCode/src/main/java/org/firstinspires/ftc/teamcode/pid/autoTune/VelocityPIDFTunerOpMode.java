package org.firstinspires.ftc.teamcode.pid.autoTune;

import org.firstinspires.ftc.teamcode.pid.autoTune.master.PIDMaster;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.library.hardware.gamepad.Button;
import org.firstinspires.ftc.teamcode.library.hardware.gamepad.GamepadEx;

import java.util.Locale;

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
 *     <li>Read the candidate gain sets from telemetry.</li>
 *     <li>Hold gamepad1.A to live-test the classic ZN candidate.</li>
 * </ol>
 */
@TeleOp(name = "Velocity PIDF Tuner", group = "FtcAutoTune")
public class VelocityPIDFTunerOpMode extends LinearOpMode {

    @Override
    public void runOpMode() {
        GamepadEx driver = new GamepadEx(gamepad1);
        // Convert RPM target to ticks/sec using: ticks/sec = (RPM * TICKS_PER_REV) / 60.0
        double targetTicksPerSec = TuningConstants.targetTicksPerSec();

        PIDMaster pid = new PIDMaster(
                hardwareMap, TuningConstants.MOTOR_NAME, TuningConstants.REVERSED, false,
                targetTicksPerSec, TuningConstants.hysteresisTicksPerSec(),
                TuningConstants.RELAY_AMPLITUDE, TuningConstants.CYCLES_TO_COLLECT, TuningConstants.CYCLES_TO_IGNORE,
                TuningConstants.RELAY_TEST_TIMEOUT_S, TuningConstants.FEEDFORWARD_TEST_POWERS,
                TuningConstants.FEEDFORWARD_SETTLE_TIME_S, TuningConstants.TUNE_INTEGRAL_TERM,
                TuningConstants.TICKS_PER_REV, TuningConstants.VELOCITY_LIVE_TEST_GAIN);

        telemetry.addLine("=== PIDF Auto Tuner (Velocity) ===");
        telemetry.addData("Motor", TuningConstants.MOTOR_NAME);
        telemetry.addData("TICKS_PER_REV", TuningConstants.TICKS_PER_REV);
        telemetry.addData("Target", String.format(Locale.US, "%.1f RPM  (%.1f ticks/s)",
                TuningConstants.TARGET_RPM, targetTicksPerSec));
        telemetry.addLine("Press START. Phase 1: relay test (automatic).");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) {
            pid.stop();
            return;
        }

        while (opModeIsActive() && !pid.isTuningComplete()) {
            pid.tuningStep(getRuntime());
            for (String line : pid.getTelemetryLines()) telemetry.addLine(line);
            telemetry.update();
        }

        if (pid.timedOut() || !pid.isTuningSuccessful()) {
            while (opModeIsActive()) {
                for (String line : pid.getTelemetryLines()) telemetry.addLine(line);
                telemetry.update();
            }
            pid.stop();
            return;
        }

        while (opModeIsActive()) {
            driver.update();
            for (String line : pid.getResultTelemetryLines()) telemetry.addLine(line);
            if (driver.isPressed(Button.A)) {
                telemetry.addLine("");
                for (String line : pid.liveTestStep(getRuntime())) telemetry.addLine(line);
            } else {
                pid.stopLiveTest();
            }
            telemetry.update();
        }
        pid.stop();
    }
}