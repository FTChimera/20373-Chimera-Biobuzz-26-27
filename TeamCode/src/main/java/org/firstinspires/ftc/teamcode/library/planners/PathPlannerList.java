package org.firstinspires.ftc.teamcode.library.planners;


import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.library.planners.paths.Utils;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public class PathPlannerList extends ArrayList<PathPlanner> implements List<PathPlanner> {
    public PathPlannerList() {
        super();
    }

    /**
     * Sets the first path planner:
     * @param startPose The starting pose to set.
     */
    public void setStartPose(Pose startPose) {
        if (size() > 0) {
            first().setStartPose(startPose);
        }
    }

    public PathPlanner last() {
        return get(size() - 1);
    }
    public PathPlanner first() {
        return get(0);
    }

    public void schedule() {
        forEach(pathPlanner -> pathPlanner.run().schedule());
    }
}
