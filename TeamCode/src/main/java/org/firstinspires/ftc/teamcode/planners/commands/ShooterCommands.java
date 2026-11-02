package org.firstinspires.ftc.teamcode.planners.commands;


import com.pedropathing.ivy.Command;

import org.firstinspires.ftc.teamcode.mechanisms.Robot;
import org.firstinspires.ftc.teamcode.util.Timer;

import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;

public class ShooterCommands {
    public static double HIVE_LINEUP_TIMEOUT_SECONDS = 1.5;
    public static double HIVE_LINEUP_TX_TOLERANCE_DEG = 1.25;

    public static Command waitForHiveLineup(Robot robot) {
        AtomicReference<Timer> timer = new AtomicReference<>();
        AtomicBoolean done = new AtomicBoolean(false);
        return Command.build()
                .setStart(() -> {
                    if (robot.vision == null) {
                        throw new IllegalStateException("Robot vision subsystem is required for hive lineup.");
                    }
                    timer.set(new Timer(TimeUnit.SECONDS));
                    done.set(false);
                })
                .setExecute(() -> {
                    if (done.get()) {
                        return;
                    }
                    if (robot.vision.hasReliableLocalization()
                            && robot.vision.isLineupWithinTolerance(
                                    robot.field.getHiveState(),
                                    HIVE_LINEUP_TX_TOLERANCE_DEG
                            )) {
                        done.set(true);
                        return;
                    }
                    if (timer.get().past(HIVE_LINEUP_TIMEOUT_SECONDS)) {
                        done.set(true);
                        robot.telemetry.addData(
                                "Hive lineup timeout",
                                "tx=%.2f state=%s",
                                robot.vision.getBestTx(robot.field.getHiveState()),
                                robot.field.getHiveState()
                        );
                    }
                })
                .setDone(done::get);
    }
    public static Command shootAllPollen(Robot robot) {
        AtomicBoolean startShoot = new AtomicBoolean(false);
        AtomicReference<Timer> timer = new AtomicReference<>();
        return Command.build()
                .requiring(robot.shooter, robot.transfer)
                .setPriority(0)
                .setStart(() -> {
                    if (!robot.shooter.hasNonzeroVelocityTarget()) {
                        throw new IllegalStateException("A nonzero shooter velocity target is required before feeding pollen");
                    }
                })
                .setExecute(() -> {
                    if (!robot.shooter.upToSpeed() || startShoot.get()) {
                        return;
                    }
                    robot.transfer.shootAll();
                    startShoot.set(true);
                    timer.set(new Timer());
                })
                .setDone(() -> startShoot.get() && timer.get().past(2000))
                .setEnd((endCondition) -> robot.transfer.stop());

    }
}
