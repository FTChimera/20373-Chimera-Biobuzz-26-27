package org.firstinspires.ftc.teamcode.planners;

import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.groups.Groups.sequential;

import androidx.annotation.NonNull;

import com.pedropathing.ivy.Command;
import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.mechanisms.Robot;
import org.firstinspires.ftc.teamcode.util.poses.PoseFactory;
import org.firstinspires.ftc.teamcode.util.poses.Pose2d;

import java.util.Objects;

public abstract class PathPlanner {
    protected Pose2d startPose;
    protected volatile Robot robot;

    public PathPlanner(Robot robot) {
        this.robot = robot;
    }
    public abstract Pose2d getEndPose();
    public abstract Command run();
    public Command total(Pose prevPose) {
        return sequential(
                instant(() -> setStartPose(prevPose)),
                instant(this::buildPaths),
                run()
        );
    }
    public abstract void buildPaths();
    public void setStartPose(Pose newPose) {this.startPose = PoseFactory.of(Objects.requireNonNull(newPose, "Start pose cannot be null"));}

    @NonNull public abstract String toString();
}
