package org.firstinspires.ftc.teamcode.library.planners;

import androidx.annotation.NonNull;

import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.Command;


public interface PathPlanner {
    Pose getEndPose();
    Command run();
    void setStartPose(Pose pose);
    void buildPaths();
    void fromPrev(PathPlanner prev);
    @NonNull String toString();
}
