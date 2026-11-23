package org.firstinspires.ftc.teamcode.planners.modules;

import androidx.annotation.NonNull;

import com.pedropathing.api.Paths;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.interpolator.Interpolator;
import com.rmsftc.cmdlib.cmd.Command;
import com.rmsftc.cmdlib.cmd.Commands;

import static org.firstinspires.ftc.teamcode.planners.commands.DrivetrainCommands.follow;

import org.firstinspires.ftc.teamcode.mechanisms.Robot;
import org.firstinspires.ftc.teamcode.planners.CommonKt;
import org.firstinspires.ftc.teamcode.planners.IntakePoses;
import org.firstinspires.ftc.teamcode.planners.PathPlanner;
import org.firstinspires.ftc.teamcode.planners.PathPoses;
import org.firstinspires.ftc.teamcode.planners.Position;
import org.firstinspires.ftc.teamcode.planners.ShootPoses;
import org.firstinspires.ftc.teamcode.planners.Timings;
import org.firstinspires.ftc.teamcode.planners.Values;
import org.firstinspires.ftc.teamcode.planners.commands.DrivetrainCommands;
import org.firstinspires.ftc.teamcode.planners.commands.ShooterCommands;
import org.firstinspires.ftc.teamcode.util.game.Field;
import org.firstinspires.ftc.teamcode.util.poses.Pose2d;

public class FromShootGardenPose {
    public static class ToIntakeFromWestFlower extends PathPlanner {
        Path toFlower;
        Path back;

        public ToIntakeFromWestFlower(Robot robot) {
            super(robot, robot.follower, robot.shooter, robot.transfer, robot.intake);
        }

        @Override
        public Pose2d getEndPose() {
            return ShootPoses.garden;
        }

        @Override
        public Position from() {
            return Position.GARDEN_SHOOT;
        }

        @Override
        public Position to() {
            return Position.GARDEN_SHOOT;
        }

        @Override
        protected Command buildCommand() {
            return Commands.sequence(
                    Commands.runOnce(robot.intake::setFullPower),
                    // Turn in place to 180 (west) first, then drive with a
                    // constant west heading -- no tangent sweep, no snap.
                    DrivetrainCommands.turnToHeadingWithTimeout(robot.follower, IntakePoses.westFlower.heading(), Timings.flowerTurnTimeoutMs),
                    Commands.parallel(DrivetrainCommands.followForIntake(robot.follower, () -> toFlower, Timings.flowerFollowTimeoutMs, Timings.flowerIntakeTime * 1000L), DrivetrainCommands.runAtCompletion(robot, Values.flowerIntakeDeployTValue, () -> robot.intake.setDeployed(true))),
                    // Drive back still facing west, then turn in place
                    // to 90 (north) for the garden shoot.
                    Commands.parallel(DrivetrainCommands.followWithTimeout(robot.follower, () -> back, Timings.flowerFollowTimeoutMs), DrivetrainCommands.runAtCompletion(robot, Values.flowerIntakeStowTValue, () -> robot.intake.setDeployed(false))),
                    DrivetrainCommands.turnToHeadingWithTimeout(robot.follower, ShootPoses.garden.heading(), Timings.flowerTurnTimeoutMs),
                    ShooterCommands.waitForHiveLineup(robot, Field.HiveState.UNTIPPED),
                    ShooterCommands.shootAllPollen(robot)
            ).finallyDo(() -> robot.intake.setDeployed(false));
        }

        @Override
        protected void buildPaths() {
            // Rect clearance to the west hive base: 9.4 in out, 12.1 in back.
            toFlower = Paths
                    .curve(startPose, PathPoses.westFlowerFromGarden, IntakePoses.westFlower)
                    .heading(Interpolator.constant(IntakePoses.westFlower.heading()));
            back = Paths
                    .curve(IntakePoses.westFlower, PathPoses.westFlowerReturn, ShootPoses.garden)
                    .heading(Interpolator.constant(IntakePoses.westFlower.heading()));
        }

        @NonNull
        @Override
        public String toString() {
            return "From Shoot Garden Pose to Intake West Flower";
        }
    }

    /**
     * North flower intake from the garden shoot pose. Extends
     * ToIntakeFromNorthFlower (loading shoot version): the return leg,
     * the run sequence, the lineup state (TIPPED) and the end pose are
     * identical, so only the outbound leg is replaced -- the approach to
     * the flower goes through the corridor between the hive bases instead
     * of the short hop from the loading shoot pose.
     */
    public static class ToIntakeNorthFlowerToLoading extends FromShootLoadingPose.ToIntakeFromNorthFlower {
        public ToIntakeNorthFlowerToLoading(Robot robot) {
            super(robot);
        }

        @Override
        public Position from() {
            return Position.GARDEN_SHOOT;
        }

        @Override
        protected void buildPaths() {
            super.buildPaths();
            // Through the corridor between the hive bases; rect clearance to
            // the west hive base: 4.2 in out. Return leg clears by 15.3 in.
            // Constant north heading -- the parent's run() already turns to
            // 90 (north) before this leg, so no tangent sweep here either.
            toFlower = Paths
                    .curve(startPose, PathPoses.northFlowerFromGarden, IntakePoses.northFlower)
                    .heading(Interpolator.constant(IntakePoses.northFlower.heading()));
        }

        @NonNull
        @Override
        public String toString() {
            return "From Shoot Garden Pose to Intake North Flower to Loading";
        }
    }

    /**
     * Garden intake that ends at the loading shoot pose instead of the
     * garden one. Extends ToIntakeGarden: the outbound leg, the run
     * sequence and the power change are identical, so only the return leg
     * is replaced (the inherited {@code back} field is reused for the
     * loading return) and the end pose + lineup state are flipped.
     */
    public static class ToIntakeGardenToLoading extends ToIntakeGarden {
        public ToIntakeGardenToLoading(Robot robot) {
            super(robot);
        }

        @Override
        public Pose2d getEndPose() {
            return ShootPoses.loading;
        }

        @Override
        public Position to() {
            return Position.LOADING_SHOOT;
        }

        @Override
        protected Field.HiveState lineupState() {
            return Field.HiveState.TIPPED;
        }

        @Override
        protected void buildPaths() {
            super.buildPaths();
            // Up the west side past the hive, then across the top to the
            // loading shoot pose. Single smooth curve; rect clearance to
            // the west hive base stays west of x=45 until above y=94
            // (curve passes (14.4, 51.5) at t=0.25, (25.8, 86) at t=0.5,
            // (40.9, 108.5) at t=0.75). Reverse tangential interpolation
            // so the robot backs up the west side instead of spinning
            // 180 degrees after the garden intake.
            back = Paths
                    .curve(IntakePoses.gardenEnd, PathPoses.gardenToLoading, ShootPoses.loading)
                    .heading(CommonKt.fastInterpolationDegraded(ShootPoses.loading.heading(), Values.gardenIntakeConstantTValue, true));
        }

        @NonNull
        @Override
        public String toString() {
            return "From Shoot Garden Pose to Intake Garden to Loading";
        }
    }

    public static class ToIntakeGarden extends PathPlanner {
        Path toGarden;
        Path back;

        public ToIntakeGarden(Robot robot) {
            super(robot, robot.follower, robot.shooter, robot.transfer, robot.intake);
        }

        @Override
        public Pose2d getEndPose() {
            return ShootPoses.garden;
        }

        @Override
        public Position from() {
            return Position.GARDEN_SHOOT;
        }

        @Override
        public Position to() {
            return Position.GARDEN_SHOOT;
        }

        /**
         * Hive state to wait for before shooting. The garden shoot wants
         * UNTIPPED; a subclass that ends at the loading shoot pose
         * overrides this to TIPPED.
         */
        protected Field.HiveState lineupState() {
            return Field.HiveState.UNTIPPED;
        }

        @Override
        protected Command buildCommand() {
            return Commands.sequence(
                    Commands.runOnce(robot.intake::setFullPower),
                    DrivetrainCommands.followForIntake(robot.follower, () -> toGarden,
                            Timings.gardenIntakeTimeoutMs, Timings.gardenIntakeTimeMs),
                    follow(robot.follower, () -> back),
                    ShooterCommands.waitForHiveLineup(robot, lineupState()),
                    ShooterCommands.shootAllPollen(robot)
            );
        }

        @Override
        protected void buildPaths() {
            toGarden = Paths
                    .curve(startPose, IntakePoses.gardenControl1, IntakePoses.gardenEnd)
                    .heading(CommonKt.fastInterpolationDegraded(IntakePoses.gardenEnd.heading(), Values.gardenIntakeConstantTValue, false));
            back = Paths
                    .curve(IntakePoses.gardenEnd, IntakePoses.gardenControl1, ShootPoses.garden)
                    .heading(CommonKt.fastInterpolationDegraded(ShootPoses.garden.heading(), Values.gardenIntakeConstantTValue, true));
        }

        @NonNull
        @Override
        public String toString() {
            return "From Shoot Garden Pose to Intake Garden";
        }
    }
}
