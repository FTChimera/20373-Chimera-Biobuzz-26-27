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

public class FromShootGardenPose {
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
