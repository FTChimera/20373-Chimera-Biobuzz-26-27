package org.firstinspires.ftc.teamcode.planners.commands;


import com.pedropathing.ivy.Command;

import org.firstinspires.ftc.teamcode.mechanisms.Robot;
import org.firstinspires.ftc.teamcode.util.Timer;

import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;

public class ShooterCommands {
    public static Command waitForHiveLineup(Robot robot) {
        return Command.build();
    }
    public static Command shootAllPollen(Robot robot) {
        AtomicBoolean startShoot = new AtomicBoolean(false);
        AtomicReference<Timer> timer = new AtomicReference<>();
        return Command.build()
                .requiring(robot.shooter, robot.transfer)
                .setPriority(0)
                .setStart(robot.transfer::slow)
                .setExecute(() -> {
                    if (!robot.shooter.upToSpeed() || startShoot.get()) {
                        return;
                    }
                    robot.transfer.shootAll();
                    startShoot.set(true);
                    timer.set(new Timer());
                })
                .setDone(() -> startShoot.get() && timer.get().past(2000))
                .setEnd((endCondition) -> robot.transfer.slow());

    }
}
