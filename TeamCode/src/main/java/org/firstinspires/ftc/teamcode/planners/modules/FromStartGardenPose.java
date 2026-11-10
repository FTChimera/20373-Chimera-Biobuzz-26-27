package org.firstinspires.ftc.teamcode.planners.modules;

import static com.pedropathing.ivy.groups.Groups.sequential;

import androidx.annotation.NonNull;

import com.pedropathing.api.Paths;
import com.pedropathing.ivy.Command;

import static org.firstinspires.ftc.teamcode.planners.commands.DrivetrainCommands.follow;
import com.pedropathing.paths.Path;

import org.firstinspires.ftc.teamcode.mechanisms.Robot;
import org.firstinspires.ftc.teamcode.util.game.Field;
import org.firstinspires.ftc.teamcode.planners.PathPlanner;
import org.firstinspires.ftc.teamcode.planners.ShootPoses;
import org.firstinspires.ftc.teamcode.planners.StartPoses;
import org.firstinspires.ftc.teamcode.planners.commands.ShooterCommands;
import org.firstinspires.ftc.teamcode.util.poses.Pose2d;

public class FromStartGardenPose {
    public static class ShootPreloads extends PathPlanner {
        Path pathToShoot;
        public ShootPreloads(Robot robot) {
            super(robot);
        }

        @Override
        public Pose2d getEndPose() {
            return ShootPoses.garden;
        }

        @Override
        public Command run() {
            return sequential(
                    follow(robot.follower, () -> pathToShoot),
                    ShooterCommands.waitForHiveLineup(robot, Field.HiveState.UNTIPPED),
                    ShooterCommands.shootAllPollen(robot)
            );
        }

        @Override
        public void buildPaths() {
            pathToShoot = Paths.line(startPose, ShootPoses.garden).linear(startPose, ShootPoses.garden);
        }

        @NonNull
        @Override
        public String toString() {
            return "Shoot Preloads at Garden";
        }
    }

    public static class ShootPreloadsToVision extends ShootPreloads {
        Path toVision;
        public ShootPreloadsToVision(Robot robot) {
            super(robot);
        }

        @Override
        public void buildPaths() {
            super.buildPaths();
            toVision = Paths.line(ShootPoses.garden, StartPoses.visionAnchorGarden).linear(ShootPoses.garden, StartPoses.visionAnchorGarden);
        }

        @Override
        public Pose2d getEndPose() {
            return StartPoses.visionAnchorGarden;
        }

        @Override
        public Command run() {
            return sequential(
                    super.run(),
                    follow(robot.follower, () -> toVision)
            );
        }

        @NonNull
        @Override
        public String toString() {
            return "Shoot Preloads to Vision at Garden";
        }
    }

    public static class ShootPreloadsToLoadingArea extends ShootPreloads {
        Path toLoading;
        public ShootPreloadsToLoadingArea(Robot robot) {
            super(robot);
        }

        @Override
        public void buildPaths() {
            super.buildPaths();
            toLoading = Paths.line(ShootPoses.garden, ShootPoses.loading).linear(ShootPoses.garden, ShootPoses.loading);
        }

        @Override
        public Pose2d getEndPose() {
            return ShootPoses.loading;
        }

        @Override
        public Command run() {
            return sequential(
                    super.run(),
                    follow(robot.follower, () -> toLoading)
            );
        }

        @NonNull
        @Override
        public String toString() {
            return "Shoot Preloads to Loading from Garden";
        }
    }
}
