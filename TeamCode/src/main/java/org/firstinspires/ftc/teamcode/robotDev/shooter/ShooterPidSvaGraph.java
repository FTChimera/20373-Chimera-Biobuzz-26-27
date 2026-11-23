package org.firstinspires.ftc.teamcode.robotDev.shooter;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Utility;

import org.firstinspires.ftc.teamcode.config.OpModeStorage;
import org.firstinspires.ftc.teamcode.mechanisms.Shooter;

/**
 * Streams the shooter PIDSVA step response to FTCDashboard so it can be
 * graphed (Dashboard -> graph: add each series by name).
 *
 * <p>What it does: spins BOTH flywheels (main + counter) through a
 * repeating 10 s step cycle and logs, every loop, the target velocity,
 * each wheel's actual velocity, each wheel's velocity error, and each
 * wheel's controller output power. The step shows the rise, the
 * steady-state error, and the zero-velocity deadband (when the target
 * returns to 0 the output is forced to 0 once the wheel is within
 * tolerance, instead of servoing on encoder noise).
 *
 * <p>Step cycle (repeats): 0-2 s target 0, 2-8 s target
 * {@link #TARGET_TICKS_PER_SECOND}, 8-10 s target 0.
 * Set {@link #TARGET_TICKS_PER_SECOND} from the dashboard before
 * running. 1341 ticks/s is ~2875 RPM, the tuner's target.
 *
 * <p>Graph in FTCDashboard by adding these series (all numeric):
 * target, mainVel, counterVel, mainErr, counterErr, mainOut, counterOut.
 * The two velocities share the target, so overlay mainVel/counterVel
 * against target to see tracking; overlay mainErr/counterErr to see
 * steady-state error; mainOut/counterOut shows the feedforward +
 * feedback power (spikes at the step, ~0 at rest via the deadband).
 */
@Config
@Utility(name = "Shooter PIDSVA Graph")
public class ShooterPidSvaGraph extends LinearOpMode {

    /** Target velocity for both flywheels, ticks/sec. Set from the dashboard. */
    public static int TARGET_TICKS_PER_SECOND = 1341;

    /** Length of one step cycle, seconds. */
    public static double CYCLE_SECONDS = 10.0;
    /** Seconds at target 0 before the step up. */
    public static double HOLD_ZERO_BEFORE_SECONDS = 2.0;
    /** Seconds at the target before stepping back to 0. */
    public static double HOLD_TARGET_SECONDS = 6.0;

    @Override
    public void runOpMode() throws InterruptedException {
        if (!OpModeStorage.initialized.get()) {
            OpModeStorage.resetToDefault();
        }

        Shooter shooter = new Shooter(hardwareMap, OpModeStorage.allianceColor.get());

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addLine("Shooter PIDSVA step graph. Set TARGET_TICKS_PER_SECOND on the dashboard, then start.");
        telemetry.addLine("Cycle: 0 (2 s) -> target (6 s) -> 0 (2 s), repeating. Graph: target, mainVel, counterVel, mainErr, counterErr, mainOut, counterOut.");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) {
            return;
        }
        double startTimeSec = getRuntime();

        try {
            while (opModeIsActive()) {
                double elapsedSec = getRuntime() - startTimeSec;
                double cycleSec = elapsedSec % CYCLE_SECONDS;
                int target;
                if (cycleSec < HOLD_ZERO_BEFORE_SECONDS) {
                    target = 0;
                } else if (cycleSec < HOLD_ZERO_BEFORE_SECONDS + HOLD_TARGET_SECONDS) {
                    target = TARGET_TICKS_PER_SECOND;
                } else {
                    target = 0;
                }

                shooter.setVelocity(target, target);
                shooter.update();

                double mainVel = shooter.mainMotor.getTicksPerSecond();
                double counterVel = shooter.counterRoller.getTicksPerSecond();
                double mainOut = shooter.mainMotor.velocityController.getOutput();
                double counterOut = shooter.counterRoller.velocityController.getOutput();

                telemetry.addData("target", target);
                telemetry.addData("mainVel", mainVel);
                telemetry.addData("counterVel", counterVel);
                telemetry.addData("mainErr", target - mainVel);
                telemetry.addData("counterErr", target - counterVel);
                telemetry.addData("mainOut", mainOut);
                telemetry.addData("counterOut", counterOut);
                telemetry.addData("upToSpeed", shooter.upToSpeed() ? 1 : 0);
                telemetry.addData("elapsedSec", elapsedSec);
                telemetry.update();
            }
        } finally {
            shooter.stopImmediately();
        }
    }
}
