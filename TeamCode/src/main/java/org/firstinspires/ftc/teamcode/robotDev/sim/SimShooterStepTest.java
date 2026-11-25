package org.firstinspires.ftc.teamcode.robotDev.sim;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.rmsftc.utils.AllianceColor;
import org.firstinspires.ftc.teamcode.mechanisms.Shooter;
import org.firstinspires.ftc.teamcode.planners.Constants;

@TeleOp(name = "Sim #3 Dual Flywheel Step", group = "Sim diagnostics")
public class SimShooterStepTest extends OpMode {
    private static final double REST_TPS = 5, REST_DWELL_S = 0.3, SETTLE_DWELL_S = 0.3;
    private static final double TIMEOUT_S = 8;
    private Shooter shooter;
    private ShooterAimMetrics.Dwell rest = new ShooterAimMetrics.Dwell();
    private ShooterAimMetrics.Dwell settle = new ShooterAimMetrics.Dwell();
    private boolean savedSotm, captured, done, stepping;
    private long phaseStart;
    private int mainTarget, counterTarget;
    private double main, counter, elapsed, mainOvershoot, counterOvershoot;
    private String result = "INIT: empty shooter; drivetrain/transfer not commanded";

    @Override public void init() {
        boolean success = false;
        try {
            savedSotm = Shooter.useSotm.get();
            captured = true;
            shooter = new Shooter(hardwareMap, AllianceColor.RED);
            // Current positive shot-table endpoints (69.5 in), not SDK velocity control.
            mainTarget = (int) Math.round(Constants.Shooter.Table.shooterMap.get(2).flywheelTicksPerSecond);
            counterTarget = (int) Math.round(Constants.Shooter.Table.counterRollerMap.get(2).flywheelTicksPerSecond);
            if (mainTarget <= 0 || counterTarget <= 0) throw new IllegalStateException("Positive step targets required.");
            shooter.stopImmediately();
            display();
            success = true;
        } finally {
            if (!success) finish("FAIL: initialization did not complete");
        }
    }

    @Override public void start() {
        phaseStart = System.nanoTime();
        result = "WAIT_REST";
    }

    @Override public void loop() {
        if (done) { display(); return; }
        boolean success = false;
        try {
            elapsed = (System.nanoTime() - phaseStart) / 1e9;
            shooter.update();
            main = shooter.mainMotor.getTicksPerSecond();
            counter = shooter.counterRoller.getTicksPerSecond();
            if (!ShooterAimMetrics.finite(elapsed, main, counter,
                    shooter.mainMotor.targetTicksPerSecond, shooter.counterRoller.targetTicksPerSecond)) {
                finish("FAIL: nonfinite wheel metric");
            } else if (elapsed >= TIMEOUT_S) {
                finish(stepping ? "FAIL: no dual-wheel sustained settle in 8 s"
                        : "FAIL: wheels did not reach rest");
            } else if (!stepping) {
                if (rest.sample(elapsed, Math.abs(main) <= REST_TPS && Math.abs(counter) <= REST_TPS, REST_DWELL_S)) {
                    shooter.setVelocity(mainTarget, counterTarget);
                    stepping = true;
                    phaseStart = System.nanoTime();
                    elapsed = 0;
                    result = "STEP";
                }
            } else {
                mainOvershoot = Math.max(mainOvershoot, main - mainTarget);
                counterOvershoot = Math.max(counterOvershoot, counter - counterTarget);
                if (settle.sample(elapsed, shooter.upToSpeed(), SETTLE_DWELL_S)) {
                    finish("PASS: both wheels continuously within tolerance for 0.3 s");
                }
            }
            display();
            success = true;
        } finally {
            if (!success) finish("FAIL: wheel API/controller did not complete");
        }
    }

    private void finish(String status) {
        result = status;
        done = true;
        try {
            if (shooter != null) shooter.stopImmediately();
        } finally {
            if (captured) Shooter.useSotm.set(savedSotm);
        }
    }

    private void display() {
        telemetry.addData("Result", result);
        telemetry.addData("Elapsed (phase s)", elapsed);
        telemetry.addData("Main target / actual / error TPS", "%d / %.2f / %.2f", mainTarget, main, mainTarget - main);
        telemetry.addData("Counter target / actual / error TPS", "%d / %.2f / %.2f", counterTarget, counter, counterTarget - counter);
        telemetry.addData("Tolerance TPS / required dwell s", "%d / %.2f",
                Constants.Shooter.SmartFlywheel.UP_TO_SPEED_TOLERANCE_TICKS_PER_SEC, SETTLE_DWELL_S);
        telemetry.addData("Rest first / dwell s", "%.3f / %.3f", rest.firstEntry, rest.longest);
        telemetry.addData("Gate first entry / sustained settle completion s", "%.3f / %.3f", settle.firstEntry, settle.settledAt);
        telemetry.addData("Current / longest continuous settle s", "%.3f / %.3f", settle.current, settle.longest);
        telemetry.addData("Main / counter overshoot TPS", "%.2f / %.2f", mainOvershoot, counterOvershoot);
        telemetry.addLine("~1.5 s is a prior expectation, NOT a physical specification; timeout 8 s.");
        telemetry.addLine("Continuous settling requires sampled gaps <=0.1 s; a longer gap restarts dwell.");
        telemetry.addLine("NaN timing = not reached. No feed / no scoring; final numbers frozen on completion.");
        telemetry.addData("Wheel outputs", done ? "OFF" : stepping ? "SMART CONTROLLER ACTIVE" : "OFF (rest wait)");
        telemetry.update();
    }

    @Override public void stop() {
        if (!done) finish("FAIL: STOP before completion");
        else finish(result);
        display();
    }
}
