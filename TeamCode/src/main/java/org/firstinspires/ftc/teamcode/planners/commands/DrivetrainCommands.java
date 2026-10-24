package org.firstinspires.ftc.teamcode.planners.commands;

import com.pedropathing.ivy.Command;
import com.pedropathing.paths.Path;

import org.firstinspires.ftc.teamcode.pedro.Follower;

public class DrivetrainCommands {
    public static Command follow(Follower follower, Path path) {
        return Command.build()
                .setStart(() -> follower.follow(path))
                .setDone(follower::atParametricEnd);
    }
}
