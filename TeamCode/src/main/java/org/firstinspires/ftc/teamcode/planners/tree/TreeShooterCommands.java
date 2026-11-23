package org.firstinspires.ftc.teamcode.planners.tree;

import com.pedropathing.api.Paths;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.interpolator.Interpolator;
import com.rmsftc.cmdlib.EndCondition;
import com.rmsftc.cmdlib.Status;
import com.rmsftc.cmdlib.cmd.Command;
import com.rmsftc.cmdlib.cmd.Commands;

import org.firstinspires.ftc.teamcode.mechanisms.Robot;
import org.firstinspires.ftc.teamcode.planners.PathPoses;
import org.firstinspires.ftc.teamcode.planners.Position;
import org.firstinspires.ftc.teamcode.planners.Timings;
import org.firstinspires.ftc.teamcode.planners.commands.DrivetrainCommands;
import org.firstinspires.ftc.teamcode.planners.commands.ShooterCommands;
import org.firstinspires.ftc.teamcode.util.poses.Pose2d;

/**
 * The tree auto's shoot behavior, replacing the
 * {@code waitForHiveLineup + shootAllPollen} ending of the shared planner
 * modules (which the other autos keep unchanged).
 *
 * <p>Semantics: when the shoot step starts, the camera is checked ONCE
 * ({@code vision.anyAllianceFiducial()}, instantaneous — no waiting, no
 * pose tolerance). If an alliance fiducial is visible the robot shoots
 * where it is; if not, it drives the under-hive path to the other shoot
 * pose and shoots anyway. The decision is made once in {@code start()}; a
 * started switch runs to completion. After shooting, the shot is recorded
 * on the blackboard (cycle count, predicted hive state, position = the
 * side actually shot from).
 *
 * <p>The under-hive path runs between the two shoot poses through the
 * {@code PathPoses.underHiveSouth}/{@code underHiveNorth} corridor
 * waypoints with a constant corridor heading (90 deg red / -90 deg blue,
 * derived from the waypoint direction so blue mirroring just works), with
 * turn-in-place steps before and after — the same pattern as the flower
 * legs. Measured with the real Pedro path geometry (17 in square robot):
 * ~2.0 in body clearance to the west hive base, body max x 70.9 in (west
 * of the x=72 midpoint).
 */
public final class TreeShooterCommands {

    private TreeShooterCommands() {
    }

    /**
     * The tree shoot step for a leg that intends to shoot from
     * {@code intendedSide} (GARDEN_SHOOT or LOADING_SHOOT).
     */
    public static Command shoot(Robot robot, TreeState state, Position intendedSide) {
        return new Command() {
            private Command inner;
            private Status innerStatus = Status.RUNNING;
            private Position shotSide;

            {
                addRequirements(robot.follower, robot.shooter, robot.transfer, robot.intake);
            }

            @Override
            public void start() {
                if (robot.vision.anyAllianceFiducial()) {
                    shotSide = intendedSide;
                    inner = ShooterCommands.shootAllPollen(robot);
                } else {
                    shotSide = TreeState.otherShootPosition(intendedSide);
                    inner = Commands.sequence(
                            switchSides(robot, intendedSide),
                            ShooterCommands.shootAllPollen(robot));
                }
                innerStatus = Status.RUNNING;
            }

            @Override
            public void loop() {
                innerStatus = inner.execute();
            }

            @Override
            public Status status() {
                return innerStatus;
            }

            @Override
            public void end(EndCondition endCondition) {
                if (endCondition == EndCondition.NATURALLY) {
                    state.recordShot(shotSide);
                } else if (inner != null) {
                    inner.invalidate();
                }
            }
        };
    }

    /**
     * Drives the under-hive path from {@code fromSide}'s shoot pose to the
     * other shoot pose: turn in place to the corridor heading, follow the
     * corridor path (no timeout, like the other cross-field legs), then
     * turn in place to the target shoot pose's heading.
     */
    public static Command switchSides(Robot robot, Position fromSide) {
        Position toSide = TreeState.otherShootPosition(fromSide);
        Pose2d fromPose = TreeState.poseFor(fromSide);
        Pose2d toPose = TreeState.poseFor(toSide);
        Pose2d wpFrom = fromSide == Position.GARDEN_SHOOT ? PathPoses.underHiveSouth : PathPoses.underHiveNorth;
        Pose2d wpTo = fromSide == Position.GARDEN_SHOOT ? PathPoses.underHiveNorth : PathPoses.underHiveSouth;
        double corridorHeading = Math.atan2(wpTo.y() - wpFrom.y(), wpTo.x() - wpFrom.x());
        Path path = Paths
                .through(fromPose, wpFrom, wpTo, toPose)
                .heading(Interpolator.constant(corridorHeading));
        return Commands.sequence(
                DrivetrainCommands.turnToHeadingWithTimeout(robot.follower, corridorHeading, Timings.flowerTurnTimeoutMs),
                DrivetrainCommands.follow(robot.follower, () -> path),
                DrivetrainCommands.turnToHeadingWithTimeout(robot.follower, toPose.heading(), Timings.flowerTurnTimeoutMs));
    }

    /**
     * The tree's reposition step: drives the under-hive path from the
     * blackboard position to the other shoot position (without shooting)
     * and updates the blackboard position on completion. The direction is
     * read from the blackboard when the command starts.
     */
    public static Command reposition(Robot robot, TreeState state) {
        return new Command() {
            private Command inner;
            private Status innerStatus = Status.RUNNING;
            private Position target;

            {
                addRequirements(robot.follower);
            }

            @Override
            public void start() {
                Position from = state.position();
                target = TreeState.otherShootPosition(from);
                inner = switchSides(robot, from);
                innerStatus = Status.RUNNING;
            }

            @Override
            public void loop() {
                innerStatus = inner.execute();
            }

            @Override
            public Status status() {
                return innerStatus;
            }

            @Override
            public void end(EndCondition endCondition) {
                if (endCondition == EndCondition.NATURALLY) {
                    state.setPosition(target);
                } else if (inner != null) {
                    inner.invalidate();
                }
            }
        };
    }
}
