package org.firstinspires.ftc.teamcode.planners.commands;

import com.rmsftc.cmdlib.EndCondition;
import com.rmsftc.cmdlib.Status;
import com.rmsftc.cmdlib.cmd.Command;
import com.rmsftc.cmdlib.cmd.Commands;
import com.rmsftc.cmdlib.cmd.WaitCommand;
import com.rmsftc.utils.Clock;

import org.firstinspires.ftc.teamcode.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.Robot;
import org.firstinspires.ftc.teamcode.planners.Constants;
import org.firstinspires.ftc.teamcode.util.Timer;
import org.firstinspires.ftc.teamcode.util.game.Field;

import java.util.concurrent.TimeUnit;

public class ShooterCommands {

    /**
     * A completed timed feed is an attempted batch, never a confirmed score.
     * SKIPPED is a zero velocity target: nothing was fed, so it is neither
     * an attempt nor an interruption — the sequence moves on with the pollen
     * still loaded.
     */
    public enum ShotOutcome {
        NOT_STARTED,
        WAITING_FOR_SPEED,
        FEEDING,
        ATTEMPTED,
        ATTEMPTED_SPINUP_TIMEOUT,
        SKIPPED,
        INTERRUPTED
    }

    static final class ShotLifecycle {
        private ShotOutcome outcome = ShotOutcome.NOT_STARTED;
        private boolean spinupTimedOut;

        ShotOutcome outcome() {
            return outcome;
        }

        boolean attempted() {
            return outcome == ShotOutcome.ATTEMPTED
                    || outcome == ShotOutcome.ATTEMPTED_SPINUP_TIMEOUT;
        }

        void start() {
            outcome = ShotOutcome.WAITING_FOR_SPEED;
            spinupTimedOut = false;
        }

        void feed(boolean timedOut) {
            if (outcome != ShotOutcome.WAITING_FOR_SPEED) {
                throw new IllegalStateException("Feeding requires waiting for shooter speed");
            }
            spinupTimedOut = timedOut;
            outcome = ShotOutcome.FEEDING;
        }

        void end(boolean interrupted) {
            if (attempted() || outcome == ShotOutcome.INTERRUPTED || outcome == ShotOutcome.SKIPPED) {
                return;
            }
            outcome = !interrupted && outcome == ShotOutcome.FEEDING
                    ? (spinupTimedOut ? ShotOutcome.ATTEMPTED_SPINUP_TIMEOUT : ShotOutcome.ATTEMPTED)
                    : ShotOutcome.INTERRUPTED;
        }

        /**
         * Ends the shot as skipped (zero velocity target): nothing was fed,
         * so this is neither an attempt nor an interruption. Only valid
         * while waiting for speed, like {@link #feed(boolean)}. Terminal:
         * later end() calls leave SKIPPED in place.
         */
        void skip() {
            if (outcome != ShotOutcome.WAITING_FOR_SPEED) {
                throw new IllegalStateException("Skipping requires waiting for shooter speed");
            }
            outcome = ShotOutcome.SKIPPED;
        }
    }

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
    public static Command waitForHiveLineupOld(Robot robot, Field.HiveState wantedState) {
        return new Command() {
            private Timer timer;
            private boolean done;

            @Override
            public void start() {
                if (robot.vision == null) {
                    throw new IllegalStateException("Robot vision subsystem is required for hive lineup.");
                }
                if (wantedState == null) {
                    throw new IllegalArgumentException("A wanted hive state is required for hive lineup.");
                }
                timer = new Timer(TimeUnit.SECONDS, robot.clock);
                done = false;
            }

            @Override
            public void loop() {
                if (done) {
                    return;
                }
                robot.vision.applyInferredHiveState(robot.field);
                if (robot.vision.hasReliableLocalization()
                        && robot.field.getHiveState() == wantedState) {
                    done = true;
                    return;
                }
                if (timer.past(Constants.Timings.hiveLineupTimeoutTime)) {
                    done = true;
                    robot.telemetry.addData(
                            "Hive lineup timeout",
                            "state=%s wanted=%s",
                            robot.field.getHiveState(),
                            wantedState
                    );
                }
            }

            @Override
            public Status status() {
                return Status.of(!done);
            }

            @Override
            public void end(EndCondition endCondition) {
            }
        };
    }

    public static ShotCommand shootAllPollen(Robot robot) {
        return new ShotCommand(robot);
    }

    public static final class ShotCommand extends Command {
        private final Robot robot;
        private final ShotLifecycle lifecycle = new ShotLifecycle();
        private Timer timer;

        private ShotCommand(Robot robot) {
            this.robot = robot;
            addRequirements(robot.shooter, robot.transfer, robot.intake);
        }

        public ShotOutcome outcome() {
            return lifecycle.outcome();
        }

        /** True only after a full timed feed, not proof that any pollen scored. */
        public boolean attempted() {
            return lifecycle.attempted();
        }

        @Override
        public void start() {
            lifecycle.start();
            // No throw on a zero velocity target: the smart-shooter daemon
            // can legitimately set a (0, 0) target for an infeasible shot
            // (e.g. SOTM over-speed degrades to 0 ticks, "do not shoot"),
            // and that must degrade, not kill the OpMode — BA does not catch
            // exceptions from start(). A zero target simply never reaches
            // up-to-speed, so loop() below times out without feeding.
            timer = new Timer(TimeUnit.MILLISECONDS, robot.clock);
        }

        @Override
        public void loop() {
            if (outcome() == ShotOutcome.FEEDING) {
                if (timer.past(Constants.Timings.shootingTimeMs)) {
                    lifecycle.end(false);
                }
                return;
            }
            if (outcome() != ShotOutcome.WAITING_FOR_SPEED) {
                return;
            }
            boolean upToSpeed = robot.shooter.upToSpeed();
            if (!upToSpeed && !timer.past(Constants.Timings.upToSpeedShooterTimeoutTimeMs)) {
                return;
            }
            if (!upToSpeed && !robot.shooter.hasNonzeroVelocityTarget()) {
                // Infeasible shot (zero target — e.g. SOTM over-speed): never
                // feed balls into a stopped flywheel. End as skipped, not
                // interrupted: the sequence moves on with the pollen still
                // loaded, and the tree must not abort on a skipped batch.
                robot.telemetry.addData("Shooter spinup timeout",
                        "WARNING: zero shooter velocity target; shot skipped, no feed");
                lifecycle.skip();
                return;
            }
            if (!upToSpeed) {
                robot.telemetry.addData("Shooter spinup timeout",
                        "WARNING: feeding without speed confirmation; batch attempt only");
            }
            robot.intake.setFullPower();
            robot.transfer.shootAll();
            timer = new Timer(TimeUnit.MILLISECONDS, robot.clock);
            lifecycle.feed(!upToSpeed);
        }

        @Override
        public Status status() {
            return Status.of(!attempted() && outcome() != ShotOutcome.INTERRUPTED
                    && outcome() != ShotOutcome.SKIPPED);
        }

        @Override
        public void end(EndCondition endCondition) {
            lifecycle.end(endCondition != EndCondition.NATURALLY);
            try {
                robot.transfer.stopImmediately();
            } finally {
                // Clear eject mode too, so its override guard cannot restart intake next update.
                robot.intake.stopEjecting(Intake.Mode.IDLE);
                robot.intake.stopImmediately();
            }
        }
    }

    public static Command waitForHiveLineup(Robot robot, Field.HiveState unused) {
        Command waitForTag = new Command() {
            private Timer timer;
            private boolean done;
            private long nanoBoundary;

            @Override
            public void start() {
                // Vision's frame observations use SYSTEM; only command durations use robot.clock.
                nanoBoundary = Clock.SYSTEM.nowNanos();
                if (robot.vision == null) {
                    throw new IllegalStateException("Robot vision subsystem is required for hive lineup.");
                }
                timer = new Timer(TimeUnit.SECONDS, robot.clock);
                done = false;
            }

            @Override
            public void loop() {
                if (done) {
                    return;
                }
                if (robot.vision.anyAllianceFiducialAfter(nanoBoundary)) {
                    done = true;
                    return;
                }
                if (timer.past(Constants.Timings.hiveLineupTimeoutTime)) {
                    done = true;
                    robot.telemetry.addData("Hive lineup timeout",
                            "WARNING: no fresh post-start alliance frame; continuing blind");
                }
            }

            @Override
            public Status status() {
                return Status.of(!done);
            }

            @Override
            public void end(EndCondition endCondition) {
            }
        };
        return Commands.sequence(
                waitForTag,
                new WaitCommand(Constants.Timings.hiveLineupExtraTimeSec * 1000L, robot.clock));
    }

    public static Command shootCmd(Robot robot, Field.HiveState hiveState) {
        return Commands.sequence(
                Commands.runOnce(() -> robot.follower.hold(robot.getPose().withHeading(robot.autoAimerCalculator.getTargetHeading()))),
                waitForHiveLineup(robot, hiveState),
                shootAllPollen(robot)
        );
    }
}
