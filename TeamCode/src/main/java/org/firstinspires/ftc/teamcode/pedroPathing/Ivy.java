package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.behaviors.EndCondition;
import com.pedropathing.ivy.pedro.PedroCommands;
import com.pedropathing.paths.HeadingInterpolator;

import java.util.Arrays;
import java.util.Objects;

public class Ivy {
    public static void run(Command command) {
        command.start();
        while (!command.done()) {
            command.execute();
        }
        command.end(EndCondition.NATURALLY);
    }

    public static Command followPath(Follower follower, Pose... poses) {
        Objects.requireNonNull(follower, "follower");
        if (poses == null || poses.length < 2) {
            throw new IllegalArgumentException(
                    "followPath requires at least 2 poses, got " + (poses == null ? "null" : poses.length));
        }
        return PedroCommands.follow(follower,
                follower.pathBuilder()
                        .addPath(new BezierCurve(poses))
                        .setGlobalLinearHeadingInterpolation(poses[0].getHeading(), poses[poses.length-1].getHeading())
                        .build()
                );
    }
}
