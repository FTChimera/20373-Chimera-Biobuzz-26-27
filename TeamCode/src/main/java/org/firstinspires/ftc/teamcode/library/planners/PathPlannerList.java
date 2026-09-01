package org.firstinspires.ftc.teamcode.library.planners;


import static com.pedropathing.ivy.groups.Groups.sequential;

import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;


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

    public Command schedule() {
        List<Command> commands = new ArrayList<>();
        forEach(pathPlanner -> commands.add(pathPlanner.asCommand()));
        Command cmd = sequential(commands.toArray(new Command[0]));
        Scheduler.schedule(cmd);
        return cmd;
    }
}
