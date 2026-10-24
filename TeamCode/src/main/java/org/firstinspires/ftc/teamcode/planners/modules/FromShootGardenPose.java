package org.firstinspires.ftc.teamcode.planners.modules;

import static com.pedropathing.ivy.groups.Groups.*;
import static com.pedropathing.ivy.commands.Commands.*;

import static org.firstinspires.ftc.teamcode.planners.commands.DrivetrainCommands.follow;

import androidx.annotation.NonNull;

import com.pedropathing.api.Paths;
import com.pedropathing.ivy.Command;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.interpolator.Interpolator;

import org.firstinspires.ftc.teamcode.mechanisms.Robot;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.planners.IntakePoses;
import org.firstinspires.ftc.teamcode.planners.PathPlanner;
import org.firstinspires.ftc.teamcode.planners.ShootPoses;
import org.firstinspires.ftc.teamcode.util.poses.Pose2d;

public class FromShootGardenPose {
    public static class ToIntakeGarden extends PathPlanner {
        Path toGarden;
        Path finishIntake;
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
                    parallel(follow(robot.follower, toGarden), setMaxPowerOnFly(0.5, 0.4))
            );
        }

        private Command setMaxPowerOnFly(double tValue, double newPower) {
            return Command.build()
                    .requiring(robot, tValue, newPower)
                    .setExecute(() -> {
                        if (robot.follower.completion() >= tValue) Constants.FOLLOWER_MAX_PATH_SPEED.set(newPower);
                    })
                    .setDone(() -> Constants.FOLLOWER_MAX_PATH_SPEED.get()==newPower);
        }

        @Override
        public void buildPaths() {
            toGarden = Paths
                    .curve(startPose, IntakePoses.gardenControl1, IntakePoses.gardenControl2, IntakePoses.gardenEnd)
                    .heading(Interpolator.tangent);
        }

        @NonNull
        @Override
        public String toString() {
            return "From Shoot Garden Pose to Intake Garden";
        }
    }
}
