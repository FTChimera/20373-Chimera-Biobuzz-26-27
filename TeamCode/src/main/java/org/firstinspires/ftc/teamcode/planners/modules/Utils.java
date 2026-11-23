package org.firstinspires.ftc.teamcode.planners.modules;

import androidx.annotation.NonNull;

import com.pedropathing.paths.Path;
import com.rmsftc.cmdlib.cmd.Command;
import com.rmsftc.cmdlib.cmd.Commands;

import static org.firstinspires.ftc.teamcode.planners.commands.DrivetrainCommands.follow;

import org.firstinspires.ftc.teamcode.mechanisms.Robot;
import org.firstinspires.ftc.teamcode.planners.PathPlanner;
import org.firstinspires.ftc.teamcode.planners.Position;
import org.firstinspires.ftc.teamcode.util.poses.PoseFactory;
import org.firstinspires.ftc.teamcode.util.poses.Pose2d;

public class Utils {
    public static class NullPlanner extends PathPlanner {
        public NullPlanner() {
            super(null);
        }

        public NullPlanner(Pose2d startPose) {
            this();
            setStartPose(startPose);
        }

        @Override
        public Pose2d getEndPose() {
            return startPose;
        }

        @Override
        public Position from() {
            return Position.ANY;
        }

        @Override
        public Position to() {
            return Position.ANY;
        }

        @Override
        protected Command buildCommand() {
            return Commands.none();
        }

        @Override
        protected void buildPaths() {

        }

        @NonNull
        @Override
        public String toString() {
            return "Null Planner";
        }
    }

    public static class WaitMs extends PathPlanner {
        double time;

        public WaitMs(Robot robot, double timeMs) {
            super(robot);
            time = timeMs;
        }

        @Override
        public Pose2d getEndPose() {
            return startPose;
        }

        @Override
        public Position from() {
            return Position.ANY;
        }

        @Override
        public Position to() {
            return Position.ANY;
        }

        @Override
        protected Command buildCommand() {
            return Commands.waitMs((long) time);
        }

        @Override
        protected void buildPaths() {

        }

        @NonNull
        @Override
        public String toString() {
            return "Wait, " + time + " ms";
        }
    }

    public static class CustomPath extends PathPlanner {
        Path path;

        public CustomPath(Robot robot, Path path) {
            super(robot, robot.follower);
            this.path = path;
        }

        @Override
        public Pose2d getEndPose() {
            return PoseFactory.of(path.endPose());
        }

        @Override
        public Position from() {
            return Position.ANY;
        }

        @Override
        public Position to() {
            return Position.ANY;
        }

        @Override
        protected Command buildCommand() {
            return follow(robot.follower, () -> path);
        }

        @Override
        protected void buildPaths() {

        }

        @NonNull
        @Override
        public String toString() {
            return "Path Follower";
        }
    }
}
