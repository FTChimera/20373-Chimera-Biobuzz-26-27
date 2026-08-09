package org.firstinspires.ftc.teamcode.library.planners;


import static com.pedropathing.ivy.commands.Commands.instant;

import androidx.annotation.NonNull;

import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.groups.Groups;


public interface PathPlanner {
    Pose getEndPose();
    Command run();
    void setStartPose(Pose pose);
    void buildPaths();
    default void fromPrev(PathPlanner prev) {
        setStartPose(prev.getEndPose());
    }
    @NonNull String toString();
    default Command asCommand() {
        return Groups.sequential(
                instant(this::buildPaths),
                run()
        );
    }
}
