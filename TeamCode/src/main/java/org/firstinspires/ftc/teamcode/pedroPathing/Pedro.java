package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.behaviors.EndCondition;
import com.pedropathing.ivy.pedro.PedroCommands;
import com.pedropathing.paths.Path;

import java.util.Arrays;
import java.util.Objects;

public class Pedro {
    public static void runIvy(Command command) {
        command.start();
        while (!command.done()) {
            command.execute();
        }
        command.end(EndCondition.NATURALLY);
    }

    public static Command ivyFollowPath(Follower follower, Pose... poses) {
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

    public static Path linear(Pose... poses) {
        if (poses.length <=1) {throw new IllegalArgumentException("Pedro Linear Path builder had 0 or 1 arguments");}
        Path result = new Path(new BezierCurve(poses));
        result.setLinearHeadingInterpolation(poses[0].getHeading(), poses[poses.length-1].getHeading());
        return result;
    }
}
