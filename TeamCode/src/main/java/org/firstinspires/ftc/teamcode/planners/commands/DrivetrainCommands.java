package org.firstinspires.ftc.teamcode.planners.commands;

import com.pedropathing.ivy.Command;
import com.pedropathing.paths.Path;

import org.firstinspires.ftc.teamcode.mechanisms.Robot;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.pedro.Follower;

public class DrivetrainCommands {
    public static Command follow(Follower follower, Path path) {
        return Command.build()
                .requiring(follower)
                .setStart(() -> follower.follow(path))
                .setDone(follower::atParametricEnd);
    }

    public static Command setMaxPowerOnFly(Robot robot, double tValue, double newPower) {
        return Command.build()
                .requiring(robot.follower)
                .setExecute(() -> {
                    if (robot.follower.completion() >= tValue) Constants.FOLLOWER_MAX_PATH_SPEED.set(newPower);
                })
                .setDone(() -> Constants.FOLLOWER_MAX_PATH_SPEED.get()==newPower);
    }
}
