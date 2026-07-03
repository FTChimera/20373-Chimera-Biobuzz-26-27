package org.firstinspires.ftc.teamcode.pid.autoTune;

import com.aaravdhawan25.pidautotuner.ftc.PIDMaster;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Auto-tunes PID gains for a run-to-position mechanism (arm, lift, turret,
 * etc.) using the relay-feedback method.
 *
 * <p>All of the actual logic (relay test, candidate computation, live test)
 * lives in {@link PIDMaster}, which comes from the {@code pidautotuner-ftc}
 * JitPack dependency. This OpMode and {@link TuningConstants} are the only two
 * files you need to copy into your own TeamCode -- everything else is
 * resolved via the library dependency.
 *
 * <h2>Setup</h2>
 * <ol>
 *     <li>Edit {@link TuningConstants} -- set {@code MOTOR_NAME},
 *         {@code POSITION_TARGET_TICKS}, and check {@code REVERSED}.</li>
 *     <li>Make sure the mechanism can safely move
 *         {@code +/- POSITION_TARGET_TICKS} from its current position without
 *         hitting hard stops -- the relay test will oscillate across that
 *         range repeatedly.</li>
 *     <li>Run this OpMode. Press start, then stand back -- the mechanism will
 *         oscillate on its own for a few seconds.</li>
 *     <li>Read the candidate gain sets off the Driver Station panelsTelemetry.</li>
 *     <li>Optional: hold gamepad1.a to live-test the "no overshoot" candidate
 *         by holding the mechanism at the target position with a real PID loop.</li>
 * </ol>
 *
 * <h2>Notes</h2>
 * <p>For mechanisms affected by gravity (arms, lifts), the relay test alone
 * will not capture a gravity feedforward term -- you may still need to add a
 * constant gravity feedforward ({@code kF * cos(angle)} or similar) on top of
 * these gains in your final code.
 */
@TeleOp(name = "Position PIDF Tuner", group = "FtcAutoTune")
public class PositionPIDTunerOpMode extends LinearOpMode {

    @Override
    public void runOpMode() {
        TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        
        PIDMaster pid = new PIDMaster(
                hardwareMap, TuningConstants.MOTOR_NAME, TuningConstants.REVERSED, true,
                TuningConstants.POSITION_TARGET_TICKS, TuningConstants.POSITION_HYSTERESIS_TICKS,
                TuningConstants.RELAY_AMPLITUDE, TuningConstants.CYCLES_TO_COLLECT, TuningConstants.CYCLES_TO_IGNORE,
                TuningConstants.RELAY_TEST_TIMEOUT_S, null, 0, TuningConstants.TUNE_INTEGRAL_TERM, 0);

        panelsTelemetry.addLine("=== PID Auto Tuner (Position) ===");
        panelsTelemetry.addData("Motor", TuningConstants.MOTOR_NAME);
        panelsTelemetry.addData("Target offset (ticks)", TuningConstants.POSITION_TARGET_TICKS);
        panelsTelemetry.addLine("Press START. The mechanism will oscillate on its own.");
        panelsTelemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        // Phase 1: relay test (runs automatically)
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

        // Phase 2: show results, optionally live-test
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