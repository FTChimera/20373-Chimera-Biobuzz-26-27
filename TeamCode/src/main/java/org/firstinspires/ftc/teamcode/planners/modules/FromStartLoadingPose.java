package org.firstinspires.ftc.teamcode.planners.modules;

import androidx.annotation.NonNull;

import com.pedropathing.api.Paths;
import com.pedropathing.paths.Path;
import com.rmsftc.cmdlib.cmd.Command;
import com.rmsftc.cmdlib.cmd.Commands;

import static org.firstinspires.ftc.teamcode.planners.commands.DrivetrainCommands.follow;

import org.firstinspires.ftc.teamcode.mechanisms.Robot;
import org.firstinspires.ftc.teamcode.planners.PathPlanner;
import org.firstinspires.ftc.teamcode.planners.Position;
import org.firstinspires.ftc.teamcode.planners.ShootPoses;
import org.firstinspires.ftc.teamcode.planners.StartPoses;
import org.firstinspires.ftc.teamcode.planners.commands.ShooterCommands;
import org.firstinspires.ftc.teamcode.util.game.Field;
import com.rmsftc.utils.math.Pose;

public class FromStartLoadingPose {
    public static class ShootPreloads extends PathPlanner {
        Path pathToShoot;

        public ShootPreloads(Robot robot) {
            super(robot, robot.follower, robot.shooter, robot.transfer, robot.intake);
        }

        @Override
        public Pose getEndPose() {
            return ShootPoses.loading;
        }

        @Override
        public Position from() {
            return Position.LOADING_START;
        }

        @Override
        public Position to() {
            return Position.LOADING_SHOOT;
        }

        @Override
        protected Command buildCommand() {
            return Commands.sequence(
                    follow(robot.follower, () -> pathToShoot),
                    ShooterCommands.waitForHiveLineup(robot, Field.HiveState.TIPPED),
                    ShooterCommands.shootAllPollen(robot)
            );
        }

        @Override
        protected void buildPaths() {
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
        protected void buildPaths() {
            super.buildPaths();
            toVision = Paths.line(ShootPoses.loading, StartPoses.visionAnchorLoading).linear(ShootPoses.loading, StartPoses.visionAnchorLoading);
        }

        @Override
        public Pose getEndPose() {
            return StartPoses.visionAnchorLoading;
        }

        @Override
        public Position to() {
            return Position.LOADING_VISION_SEARCH;
        }

        @Override
        protected Command buildCommand() {
            return Commands.sequence(
                    super.buildCommand(),
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
        protected void buildPaths() {
            super.buildPaths();
            toGarden = robot.field.generateTransitPath(ShootPoses.loading, ShootPoses.garden);
        }

        @Override
        public Pose getEndPose() {
            return ShootPoses.garden;
        }

        @Override
        public Position to() {
            return Position.GARDEN_SHOOT;
        }

        @Override
        protected Command buildCommand() {
            return Commands.sequence(
                    super.buildCommand(),
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
