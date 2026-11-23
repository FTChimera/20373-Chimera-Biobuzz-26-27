package org.firstinspires.ftc.teamcode.planners.commands;

import com.rmsftc.cmdlib.EndCondition;
import com.rmsftc.cmdlib.Status;
import com.rmsftc.cmdlib.cmd.Command;
import com.rmsftc.cmdlib.cmd.Commands;

import org.firstinspires.ftc.teamcode.mechanisms.Robot;
import org.firstinspires.ftc.teamcode.planners.Timings;
import org.firstinspires.ftc.teamcode.util.Timer;
import org.firstinspires.ftc.teamcode.util.game.Field;

import java.util.concurrent.TimeUnit;

public class ShooterCommands {

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
                timer = new Timer(TimeUnit.SECONDS);
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
                if (timer.past(Timings.hiveLineupTimeoutTime)) {
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

    public static Command shootAllPollen(Robot robot) {
        return new Command() {
            private Timer timer;
            private boolean startShoot;

            {
                addRequirements(robot.shooter, robot.transfer, robot.intake);
            }

            @Override
            public void start() {
                if (!robot.shooter.hasNonzeroVelocityTarget()) {
                    throw new IllegalStateException("A nonzero shooter velocity target is required before feeding pollen");
                }
                timer = new Timer();
                startShoot = false;
            }

            @Override
            public void loop() {
                if (!robot.shooter.upToSpeed() && !timer.past(Timings.upToSpeedShooterTimeoutTimeMs)) {
                    return;
                }
                if (startShoot) return;
                robot.intake.setFullPower();
                robot.transfer.shootAll();
                timer = new Timer();
                startShoot = true;
            }

            @Override
            public Status status() {
                return Status.of(!(startShoot && timer.past(Timings.shootingTimeMs)));
            }

            @Override
            public void end(EndCondition endCondition) {
                robot.transfer.stop();
            }
        };
    }

    public static Command waitForHiveLineup(Robot robot, Field.HiveState unused) {
        Command waitForTag = new Command() {
            private Timer timer;
            private boolean done;

            @Override
            public void start() {
                timer = new Timer(TimeUnit.SECONDS);
                done = false;
            }

            @Override
            public void loop() {
                if (done) {
                    return;
                }
                if (robot.vision.anyAllianceFiducial()) {
                    done = true;
                    return;
                }
                if (timer.past(Timings.hiveLineupTimeoutTime)) {
                    done = true;
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
                Commands.waitMs(Timings.hiveLineupExtraTimeSec * 1000L));
    }

    public static Command shootCmd(Robot robot, Field.HiveState hiveState) {
        return Commands.sequence(
                Commands.runOnce(() -> robot.follower.hold(robot.getPose().withHeading(robot.autoAimerCalculator.getTargetHeading()))),
                waitForHiveLineup(robot, hiveState),
                shootAllPollen(robot)
        );
    }
}
