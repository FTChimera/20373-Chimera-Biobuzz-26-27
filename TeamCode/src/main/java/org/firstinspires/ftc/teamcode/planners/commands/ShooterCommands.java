package org.firstinspires.ftc.teamcode.planners.commands;


import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.groups.Groups.sequential;

import com.pedropathing.ivy.Command;

import org.firstinspires.ftc.teamcode.mechanisms.Robot;
import org.firstinspires.ftc.teamcode.util.Timer;
import org.firstinspires.ftc.teamcode.util.game.Field;

import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;

public class ShooterCommands {
    public static double HIVE_LINEUP_TIMEOUT_SECONDS = 1.5;

    /**
     * Hive lineup is about the hive STATE (tipped / untipped), not aiming:
     * waits until the hive is actually in {@code wantedState} - the state
     * whose scoring face is on the side the robot is launching from
     * (garden shoot -> UNTIPPED, loading shoot -> TIPPED, for either
     * alliance). Each cycle applies Vision's inferred state to the field,
     * so the shooter and auto-aimer aim at the face that will actually
     * score. There is no tx check here - tx is for aiming, and aiming is
     * the auto-aligner's job. If the hive never reaches the wanted state
     * (or Vision cannot see the tags), the last known state stands and the
     * command ends on timeout.
     */
    public static Command waitForHiveLineup(Robot robot, Field.HiveState wantedState) {
        AtomicReference<Timer> timer = new AtomicReference<>();
        AtomicBoolean done = new AtomicBoolean(false);
        return Command.build()
                .setStart(() -> {
                    if (robot.vision == null) {
                        throw new IllegalStateException("Robot vision subsystem is required for hive lineup.");
                    }
                    if (wantedState == null) {
                        throw new IllegalArgumentException("A wanted hive state is required for hive lineup.");
                    }
                    timer.set(new Timer(TimeUnit.SECONDS));
                    done.set(false);
                })
                .setExecute(() -> {
                    if (done.get()) {
                        return;
                    }
                    robot.vision.applyInferredHiveState(robot.field);
                    if (robot.vision.hasReliableLocalization()
                            && robot.field.getHiveState() == wantedState) {
                        done.set(true);
                        return;
                    }
                    if (timer.get().past(HIVE_LINEUP_TIMEOUT_SECONDS)) {
                        done.set(true);
                        robot.telemetry.addData(
                                "Hive lineup timeout",
                                "state=%s wanted=%s",
                                robot.field.getHiveState(),
                                wantedState
                        );
                    }
                })
                .setDone(done::get);
    }
    public static Command shootAllPollen(Robot robot) {
        AtomicBoolean startShoot = new AtomicBoolean(false);
        AtomicReference<Timer> timer = new AtomicReference<>();
        return Command.build()
                .requiring(robot.shooter, robot.transfer, robot.intake)
                .setPriority(0)
                .setStart(() -> {
                    if (!robot.shooter.hasNonzeroVelocityTarget()) {
                        throw new IllegalStateException("A nonzero shooter velocity target is required before feeding pollen");
                    }
                    robot.intake.setFullPower();
                })
                .setExecute(() -> {
                    if (!robot.shooter.upToSpeed() || startShoot.get()) {
                        return;
                    }
                    robot.transfer.shootAll();
                    startShoot.set(true);
                    timer.set(new Timer());
                })
                .setDone(() -> startShoot.get() && timer.get().past(5000))
                .setEnd((endCondition) -> robot.transfer.stop());

    }


    public static Command shootCmd(Robot robot, Field.HiveState hiveState) {
        return sequential(
                instant(() -> robot.follower.hold(robot.getPose().withHeading(robot.autoAimerCalculator.getTargetHeading()))),
                waitForHiveLineup(robot, hiveState),
                shootAllPollen(robot)
        );
    }
}
