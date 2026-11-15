package org.firstinspires.ftc.teamcode.planners.modules;

import static com.pedropathing.ivy.groups.Groups.*;
import static com.pedropathing.ivy.commands.Commands.*;

import static org.firstinspires.ftc.teamcode.planners.commands.DrivetrainCommands.follow;

import androidx.annotation.NonNull;

import com.pedropathing.api.Paths;
import com.pedropathing.ivy.Command;
import com.pedropathing.paths.Path;

import org.firstinspires.ftc.teamcode.mechanisms.Robot;
import org.firstinspires.ftc.teamcode.util.game.Field;
import org.firstinspires.ftc.teamcode.planners.CommonKt;
import org.firstinspires.ftc.teamcode.planners.IntakePoses;
import org.firstinspires.ftc.teamcode.planners.PathPlanner;
import org.firstinspires.ftc.teamcode.planners.ShootPoses;
import org.firstinspires.ftc.teamcode.planners.Timings;
import org.firstinspires.ftc.teamcode.planners.Values;
import org.firstinspires.ftc.teamcode.planners.commands.DrivetrainCommands;
import org.firstinspires.ftc.teamcode.planners.commands.ShooterCommands;
import org.firstinspires.ftc.teamcode.util.poses.Pose2d;
import org.firstinspires.ftc.teamcode.util.poses.PoseFactory;

public class FromShootGardenPose {
    public static class ToIntakeFromWestFlower extends PathPlanner {
        Path toFlower;
        Path back;
        public ToIntakeFromWestFlower(Robot robot) {
            super(robot);
        }

        @Override
        public Pose2d getEndPose() {
            return ShootPoses.garden;
        }

        @Override
        public Command run() {
            return sequential(
                    instant(robot.intake::setFullPower),
                    parallel(follow(robot.follower, () -> toFlower), DrivetrainCommands.runAtCompletion(robot, Values.flowerIntakeDeployTValue, () -> robot.intake.setDeployed(true))),
                    waitMs(Timings.flowerIntakeTime * 1000),
                    parallel(follow(robot.follower, () -> back), DrivetrainCommands.runAtCompletion(robot, Values.flowerIntakeStowTValue, () -> robot.intake.setDeployed(false))),
                    ShooterCommands.waitForHiveLineup(robot, Field.HiveState.UNTIPPED),
                    ShooterCommands.shootAllPollen(robot)
            );
        }

        @Override
        public void buildPaths() {
            // Rect clearance to the west hive base: 9.4 in out, 12.1 in back.
            toFlower = Paths
                    .curve(startPose, PoseFactory.of(36.0, 36.0), IntakePoses.westFlower)
                    .heading(CommonKt.fastInterpolationDegraded(IntakePoses.westFlower.heading(), Values.gardenIntakeConstantTValue, false));
            back = Paths
                    .curve(IntakePoses.westFlower, PoseFactory.of(36.0, 30.0), ShootPoses.garden)
                    .heading(CommonKt.fastInterpolationDegraded(ShootPoses.garden.heading(), Values.gardenIntakeConstantTValue, true));
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
        public void buildPaths() {
            super.buildPaths();
            // Through the corridor between the hive bases; rect clearance to
            // the west hive base: 4.2 in out. Return leg clears by 15.3 in.
            toFlower = Paths
                    .curve(startPose, PoseFactory.of(78.0, 72.0), IntakePoses.northFlower)
                    .heading(CommonKt.fastInterpolationDegraded(IntakePoses.northFlower.heading(), Values.gardenIntakeConstantTValue, false));
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
        protected Field.HiveState lineupState() {
            return Field.HiveState.TIPPED;
        }

        @Override
        public void buildPaths() {
            super.buildPaths();
            // Up the west side past the hive, then across the top to the
            // loading shoot pose. Single smooth curve; rect clearance to
            // the west hive base stays west of x=45 until above y=94
            // (curve passes (14.4, 51.5) at t=0.25, (25.8, 86) at t=0.5,
            // (40.9, 108.5) at t=0.75). Reverse tangential interpolation
            // so the robot backs up the west side instead of spinning
            // 180 degrees after the garden intake.
            back = Paths
                    .curve(IntakePoses.gardenEnd, PoseFactory.of(18.0, 110.0), ShootPoses.loading)
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
            super(robot);
        }

        @Override
        public Pose2d getEndPose() {
            return ShootPoses.garden;
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
        public Command run() {
            return sequential(
                    instant(robot.intake::setFullPower),
                    parallel(follow(robot.follower, () -> toGarden), DrivetrainCommands.setMaxPowerOnFly(robot, Values.gardenIntakeMaxPowerSetChangeTValue, Values.gardenIntakeMaxPowerSetChangeNewPower)),
                    waitMs(Timings.gardenIntakeTimeMs),
                    follow(robot.follower, () -> back),
                    ShooterCommands.waitForHiveLineup(robot, lineupState()),
                    ShooterCommands.shootAllPollen(robot)
            );
        }

        @Override
        public void buildPaths() {
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
