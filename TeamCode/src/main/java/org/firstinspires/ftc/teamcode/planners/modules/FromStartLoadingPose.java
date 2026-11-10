package org.firstinspires.ftc.teamcode.planners.modules;

import static com.pedropathing.ivy.groups.Groups.sequential;

import androidx.annotation.NonNull;

import com.pedropathing.api.Paths;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.pedro.PedroCommands;
import com.pedropathing.paths.Path;

import static org.firstinspires.ftc.teamcode.planners.commands.DrivetrainCommands.follow;

import org.firstinspires.ftc.teamcode.mechanisms.Robot;
import org.firstinspires.ftc.teamcode.util.game.Field;
import org.firstinspires.ftc.teamcode.planners.PathPlanner;
import org.firstinspires.ftc.teamcode.planners.ShootPoses;
import org.firstinspires.ftc.teamcode.planners.StartPoses;
import org.firstinspires.ftc.teamcode.planners.commands.ShooterCommands;
import org.firstinspires.ftc.teamcode.util.poses.Pose2d;

public class FromStartLoadingPose {
    public static class ShootPreloads extends PathPlanner {
        Path pathToShoot;
        public ShootPreloads(Robot robot) {
            super(robot);
        }

        @Override
        public Pose2d getEndPose() {
            return ShootPoses.loading;
        }

        @Override
        public Command run() {
            return sequential(
                    follow(robot.follower, () -> pathToShoot),
                    ShooterCommands.waitForHiveLineup(robot, Field.HiveState.TIPPED),
                    ShooterCommands.shootAllPollen(robot)
            );
        }

        @Override
        public void buildPaths() {
            pathToShoot = Paths.line(startPose, ShootPoses.loading).linear(startPose, ShootPoses.loading);
        }

        @NonNull
        @Override
        public String toString() {
            return "Shoot Preloads at Loading";
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
            toVision = Paths.line(ShootPoses.loading, StartPoses.visionAnchorLoading).linear(ShootPoses.loading, StartPoses.visionAnchorLoading);
        }

        @Override
        public Pose2d getEndPose() {
            return StartPoses.visionAnchorLoading;
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
            return "Shoot Preloads to Vision at Loading";
        }
    }

    public static class ShootPreloadsToGardenArea extends ShootPreloads {
        Path toGarden;
        public ShootPreloadsToGardenArea(Robot robot) {
            super(robot);
        }

        @Override
        public void buildPaths() {
            super.buildPaths();
            toGarden = Paths.line(ShootPoses.loading, ShootPoses.garden).linear(ShootPoses.loading, ShootPoses.garden);
        }

        @Override
        public Pose2d getEndPose() {
            return ShootPoses.garden;
        }

        @Override
        public Command run() {
            return sequential(
                    super.run(),
                    follow(robot.follower, () -> toGarden)
            );
        }

        @NonNull
        @Override
        public String toString() {
            return "Shoot Preloads to Garden from Loading";
        }
    }
}
