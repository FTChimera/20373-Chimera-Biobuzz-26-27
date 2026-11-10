package org.firstinspires.ftc.teamcode.planners.modules;

import static com.pedropathing.ivy.groups.Groups.*;
import static com.pedropathing.ivy.commands.Commands.*;

import static org.firstinspires.ftc.teamcode.planners.commands.DrivetrainCommands.follow;

import androidx.annotation.NonNull;

import com.pedropathing.api.Paths;
import com.pedropathing.ivy.Command;
import com.pedropathing.paths.Path;

import org.firstinspires.ftc.teamcode.mechanisms.Robot;
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
                    ShooterCommands.waitForHiveLineup(robot),
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

    public static class ToIntakeNorthFlowerToLoading extends PathPlanner {
        Path toFlower;
        Path back;
        public ToIntakeNorthFlowerToLoading(Robot robot) {
            super(robot);
        }

        @Override
        public Pose2d getEndPose() {
            return ShootPoses.loading;
        }

        @Override
        public Command run() {
            return sequential(
                    instant(robot.intake::setFullPower),
                    parallel(follow(robot.follower, () -> toFlower), DrivetrainCommands.runAtCompletion(robot, Values.flowerIntakeDeployTValue, () -> robot.intake.setDeployed(true))),
                    waitMs(Timings.flowerIntakeTime * 1000),
                    parallel(follow(robot.follower, () -> back), DrivetrainCommands.runAtCompletion(robot, Values.flowerIntakeStowTValue, () -> robot.intake.setDeployed(false))),
                    ShooterCommands.waitForHiveLineup(robot),
                    ShooterCommands.shootAllPollen(robot)
            );
        }

        @Override
        public void buildPaths() {
            // Through the corridor between the hive bases; rect clearance to
            // the west hive base: 4.2 in out. Return leg clears by 15.3 in.
            toFlower = Paths
                    .curve(startPose, PoseFactory.of(78.0, 72.0), IntakePoses.northFlower)
                    .heading(CommonKt.fastInterpolationDegraded(IntakePoses.northFlower.heading(), Values.gardenIntakeConstantTValue, false));
            back = Paths
                    .curve(IntakePoses.northFlower, PoseFactory.of(54.0, 127.0), ShootPoses.loading)
                    .heading(CommonKt.fastInterpolationDegraded(ShootPoses.loading.heading(), Values.gardenIntakeConstantTValue, true));
        }

        @NonNull
        @Override
        public String toString() {
            return "From Shoot Garden Pose to Intake North Flower to Loading";
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

        @Override
        public Command run() {
            return sequential(
                    instant(robot.intake::setFullPower),
                    parallel(follow(robot.follower, () -> toGarden), DrivetrainCommands.setMaxPowerOnFly(robot, Values.gardenIntakeMaxPowerSetChangeTValue, Values.gardenIntakeMaxPowerSetChangeNewPower)),
                    waitMs(Timings.gardenIntakeTimeMs),
                    follow(robot.follower, () -> back),
                    ShooterCommands.waitForHiveLineup(robot),
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
