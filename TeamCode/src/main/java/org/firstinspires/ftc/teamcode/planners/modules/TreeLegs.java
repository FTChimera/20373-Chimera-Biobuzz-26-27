package org.firstinspires.ftc.teamcode.planners.modules;

import androidx.annotation.NonNull;

import com.rmsftc.cmdlib.cmd.Command;
import com.rmsftc.cmdlib.cmd.Commands;

import org.firstinspires.ftc.teamcode.mechanisms.Robot;
import org.firstinspires.ftc.teamcode.planners.Constants;
import org.firstinspires.ftc.teamcode.planners.PathPlanner;
import org.firstinspires.ftc.teamcode.planners.Position;
import org.firstinspires.ftc.teamcode.planners.commands.DrivetrainCommands;
import org.firstinspires.ftc.teamcode.planners.tree.TreeShooterCommands;
import org.firstinspires.ftc.teamcode.planners.tree.TreeState;

/**
 * Tree-auto versions of the shared planner legs. Each subclass reuses its
 * parent's {@code buildPaths()} geometry unchanged and overrides only
 * {@code buildCommand()}: the pre-shoot steps are the parent's, but the
 * ending {@code waitForHiveLineup + shootAllPollen} is replaced by the
 * tree shoot step ({@link TreeShooterCommands#shoot}), which checks the
 * latest camera result after bounded physical settling (plain tag rule:
 * any alliance tag shoots from this side, else one measured side switch).
 * At most one measured side switch is allowed; blind attempts remain explicit.
 * The shared modules (and the autos that use them) are
 * untouched.
 *
 * <p>These live in {@code planners.modules} rather than
 * {@code planners.tree} because the parents' path fields are
 * package-private.
 */
public final class TreeLegs {

    private TreeLegs() {
    }

    /** Creates the tree leg for {@code legId}. */
    public static PathPlanner create(TreeState.LegId legId, Robot robot, TreeState state) {
        switch (legId) {
            case SHOOT_PRELOADS: return new ShootPreloads(robot, state);
            case GARDEN_TO_LOADING: return new GardenToLoading(robot, state);
            case NORTH_FLOWER: return new NorthFlower(robot, state);
            case LOADING_TO_GARDEN: return new LoadingToGarden(robot, state);
            case WEST_FLOWER: return new WestFlower(robot, state);
            case GARDEN: return new Garden(robot, state);
            case WEST_FLOWER_TO_GARDEN: return new WestFlowerToGarden(robot, state);
            default: throw new IllegalArgumentException("Unknown leg: " + legId);
        }
    }

    public static class ShootPreloads extends FromStartGardenPose.ShootPreloads {
        private final TreeState state;

        public ShootPreloads(Robot robot, TreeState state) {
            super(robot);
            this.state = state;
        }

        @Override
        protected Command buildCommand() {
            return Commands.sequence(
                    DrivetrainCommands.followWithTimeout(robot.follower, () -> pathToShoot, Constants.Tree.Shoot.SHOOT_APPROACH_TIMEOUT_MS),
                    TreeShooterCommands.shoot(robot, state, Position.GARDEN_SHOOT));
        }

        @NonNull
        @Override
        public String toString() {
            return "Shoot Preloads at Garden (tree)";
        }
    }

    public static class GardenToLoading extends FromShootGardenPose.ToIntakeGardenToLoading {
        private final TreeState state;

        public GardenToLoading(Robot robot, TreeState state) {
            super(robot);
            this.state = state;
        }

        @Override
        protected Command buildCommand() {
            return Commands.sequence(
                    Commands.runOnce(robot.intake::setFullPower),
                    DrivetrainCommands.followForIntake(robot.follower, () -> toGarden, Constants.Timings.gardenIntakeTimeoutMs, Constants.Timings.gardenIntakeTimeMs),
                    DrivetrainCommands.followWithTimeout(robot.follower, () -> back, Constants.Tree.Shoot.SHOOT_APPROACH_TIMEOUT_MS),
                    TreeShooterCommands.shoot(robot, state, Position.LOADING_SHOOT, TreeState.IntakeArea.GARDEN));
        }

        @NonNull
        @Override
        public String toString() {
            return "From Shoot Garden Pose to Intake Garden to Loading (tree)";
        }
    }

    public static class Garden extends FromShootGardenPose.ToIntakeGarden {
        private final TreeState state;

        public Garden(Robot robot, TreeState state) {
            super(robot);
            this.state = state;
        }

        @Override
        protected Command buildCommand() {
            return Commands.sequence(
                    Commands.runOnce(robot.intake::setFullPower),
                    DrivetrainCommands.followForIntake(robot.follower, () -> toGarden, Constants.Timings.gardenIntakeTimeoutMs, Constants.Timings.gardenIntakeTimeMs),
                    DrivetrainCommands.followWithTimeout(robot.follower, () -> back, Constants.Tree.Shoot.SHOOT_APPROACH_TIMEOUT_MS),
                    TreeShooterCommands.shoot(robot, state, Position.GARDEN_SHOOT, TreeState.IntakeArea.GARDEN));
        }

        @NonNull
        @Override
        public String toString() {
            return "From Shoot Garden Pose to Intake Garden (tree)";
        }
    }

    public static class WestFlower extends FromShootGardenPose.ToIntakeFromWestFlower {
        private final TreeState state;

        public WestFlower(Robot robot, TreeState state) {
            super(robot);
            this.state = state;
        }

        @Override
        protected Command buildCommand() {
            return Commands.sequence(
                    Commands.runOnce(robot.intake::setFullPower),
                    DrivetrainCommands.turnToHeadingWithTimeout(robot.follower, Constants.IntakePoses.westFlower.heading(), Constants.Timings.flowerTurnTimeoutMs),
                    Commands.parallel(DrivetrainCommands.followForIntake(robot.follower, () -> toFlower, Constants.Timings.flowerFollowTimeoutMs, Constants.Timings.flowerIntakeTime * 1000L), DrivetrainCommands.runAtCompletion(robot, Constants.Values.flowerIntakeDeployTValue, () -> robot.intake.setDeployed(true))),
                    Commands.parallel(DrivetrainCommands.followWithTimeout(robot.follower, () -> back, Constants.Timings.flowerFollowTimeoutMs), DrivetrainCommands.runAtCompletion(robot, Constants.Values.flowerIntakeStowTValue, () -> robot.intake.setDeployed(false))),
                    // Final align turn, mirroring the fixed leg: the return
                    // leg ends facing the flower, so turn to the shoot
                    // heading before shooting instead of shooting while
                    // still rotating.
                    DrivetrainCommands.turnToHeadingWithTimeout(robot.follower, Constants.ShootPoses.garden.heading(), Constants.Timings.flowerTurnTimeoutMs),
                    TreeShooterCommands.shoot(robot, state, Position.GARDEN_SHOOT, TreeState.IntakeArea.WEST_FLOWER))
                    .finallyDo(() -> robot.intake.setDeployed(false));
        }

        @NonNull
        @Override
        public String toString() {
            return "From Shoot Garden Pose to Intake West Flower (tree)";
        }
    }

    public static class NorthFlower extends FromShootLoadingPose.ToIntakeFromNorthFlower {
        private final TreeState state;

        public NorthFlower(Robot robot, TreeState state) {
            super(robot);
            this.state = state;
        }

        @Override
        protected Command buildCommand() {
            return Commands.sequence(
                    Commands.runOnce(robot.intake::setFullPower),
                    DrivetrainCommands.turnToHeadingWithTimeout(robot.follower, Constants.IntakePoses.northFlower.heading(), Constants.Timings.flowerTurnTimeoutMs),
                    Commands.parallel(DrivetrainCommands.followForIntake(robot.follower, () -> toFlower, Constants.Timings.flowerFollowTimeoutMs, Constants.Timings.flowerIntakeTime * 1000L), DrivetrainCommands.runAtCompletion(robot, Constants.Values.flowerIntakeDeployTValue, () -> robot.intake.setDeployed(true))),
                    Commands.parallel(DrivetrainCommands.followWithTimeout(robot.follower, () -> back, Constants.Timings.flowerFollowTimeoutMs), DrivetrainCommands.runAtCompletion(robot, Constants.Values.flowerIntakeStowTValue, () -> robot.intake.setDeployed(false))),
                    // Final align turn, mirroring the fixed leg: the return
                    // leg ends facing north (90 deg), so turn to the loading
                    // shoot heading (-90 deg) before shooting instead of
                    // shooting while still rotating.
                    DrivetrainCommands.turnToHeadingWithTimeout(robot.follower, Constants.ShootPoses.loading.heading(), Constants.Timings.flowerTurnTimeoutMs),
                    TreeShooterCommands.shoot(robot, state, Position.LOADING_SHOOT, TreeState.IntakeArea.NORTH_FLOWER))
                    .finallyDo(() -> robot.intake.setDeployed(false));
        }

        @NonNull
        @Override
        public String toString() {
            return "From Shoot Loading Pose to Intake North Flower (tree)";
        }
    }

    public static class LoadingToGarden extends FromShootLoadingPose.ToIntakeLoadingToGarden {
        private final TreeState state;

        public LoadingToGarden(Robot robot, TreeState state) {
            super(robot);
            this.state = state;
        }

        @Override
        protected Command buildCommand() {
            return Commands.sequence(
                    Commands.runOnce(robot.intake::setFullPower),
                    DrivetrainCommands.followForIntake(robot.follower, () -> toWall, Constants.Timings.gardenIntakeTimeoutMs, Constants.Timings.gardenIntakeTimeMs),
                    DrivetrainCommands.followWithTimeout(robot.follower, () -> toGarden, Constants.Tree.Shoot.SHOOT_APPROACH_TIMEOUT_MS),
                    TreeShooterCommands.shoot(robot, state, Position.GARDEN_SHOOT, TreeState.IntakeArea.LOADING_PRELOADS));
        }

        @NonNull
        @Override
        public String toString() {
            return "From Shoot Loading Pose to Intake Loading Preloads to Garden (tree)";
        }
    }

    public static class WestFlowerToGarden extends FromShootLoadingPose.ToIntakeWestFlowerToGarden {
        private final TreeState state;

        public WestFlowerToGarden(Robot robot, TreeState state) {
            super(robot);
            this.state = state;
        }

        @Override
        protected Command buildCommand() {
            return Commands.sequence(
                    Commands.runOnce(robot.intake::setFullPower),
                    DrivetrainCommands.turnToHeadingWithTimeout(robot.follower, Constants.IntakePoses.westFlower.heading(), Constants.Timings.flowerTurnTimeoutMs),
                    Commands.parallel(DrivetrainCommands.followForIntake(robot.follower, () -> toFlower, Constants.Timings.flowerFollowTimeoutMs, Constants.Timings.flowerIntakeTime * 1000L), DrivetrainCommands.runAtCompletion(robot, Constants.Values.flowerIntakeDeployTValue, () -> robot.intake.setDeployed(true))),
                    Commands.parallel(DrivetrainCommands.followWithTimeout(robot.follower, () -> back, Constants.Timings.flowerFollowTimeoutMs), DrivetrainCommands.runAtCompletion(robot, Constants.Values.flowerIntakeStowTValue, () -> robot.intake.setDeployed(false))),
                    // Final align turn, mirroring the fixed leg (which turns
                    // to the garden shoot heading before shooting): the
                    // return leg ends facing the flower, so turn before
                    // shooting instead of shooting while still rotating.
                    DrivetrainCommands.turnToHeadingWithTimeout(robot.follower, Constants.ShootPoses.garden.heading(), Constants.Timings.flowerTurnTimeoutMs),
                    TreeShooterCommands.shoot(robot, state, Position.GARDEN_SHOOT, TreeState.IntakeArea.WEST_FLOWER))
                    .finallyDo(() -> robot.intake.setDeployed(false));
        }

        @NonNull
        @Override
        public String toString() {
            return "From Shoot Loading Pose to Intake West Flower to Garden (tree)";
        }
    }
}
