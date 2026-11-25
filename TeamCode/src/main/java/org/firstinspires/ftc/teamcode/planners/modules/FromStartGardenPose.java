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
import org.firstinspires.ftc.teamcode.planners.Constants;
import org.firstinspires.ftc.teamcode.planners.commands.ShooterCommands;
import org.firstinspires.ftc.teamcode.util.game.Field;
import com.rmsftc.utils.math.Pose;

public class FromStartGardenPose {
    public static class ShootPreloads extends PathPlanner {
        Path pathToShoot;

        public ShootPreloads(Robot robot) {
            super(robot, robot.follower, robot.shooter, robot.transfer, robot.intake);
        }

        @Override
        public Pose getEndPose() {
            return Constants.ShootPoses.garden;
        }

        @Override
        public Position from() {
            return Position.GARDEN_START;
        }

        @Override
        public Position to() {
            return Position.GARDEN_SHOOT;
        }

        @Override
        protected Command buildCommand() {
            return Commands.sequence(
                    follow(robot.follower, () -> pathToShoot, robot.clock),
                    ShooterCommands.waitForHiveLineup(robot, Field.HiveState.UNTIPPED),
                    ShooterCommands.shootAllPollen(robot)
            );
        }

        @Override
        protected void buildPaths() {
            pathToShoot = Paths.line(startPose, Constants.ShootPoses.garden).linear(startPose, Constants.ShootPoses.garden);
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
        protected void buildPaths() {
            super.buildPaths();
            toVision = Paths.line(Constants.ShootPoses.garden, Constants.StartPoses.visionAnchorGarden).linear(Constants.ShootPoses.garden, Constants.StartPoses.visionAnchorGarden);
        }

        @Override
        public Pose getEndPose() {
            return Constants.StartPoses.visionAnchorGarden;
        }

        @Override
        public Position to() {
            return Position.GARDEN_VISION_SEARCH;
        }

        @Override
        protected Command buildCommand() {
            return Commands.sequence(
                    super.buildCommand(),
                    follow(robot.follower, () -> toVision, robot.clock)
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
        protected void buildPaths() {
            super.buildPaths();
            toLoading = robot.field.generateTransitPath(Constants.ShootPoses.garden, Constants.ShootPoses.loading);
        }

        @Override
        public Pose getEndPose() {
            return Constants.ShootPoses.loading;
        }

        @Override
        public Position to() {
            return Position.LOADING_SHOOT;
        }

        @Override
        protected Command buildCommand() {
            return Commands.sequence(
                    super.buildCommand(),
                    follow(robot.follower, () -> toLoading, robot.clock)
            );
        }

        @NonNull
        @Override
        public String toString() {
            return "Shoot Preloads to Loading from Garden";
        }
    }
}
