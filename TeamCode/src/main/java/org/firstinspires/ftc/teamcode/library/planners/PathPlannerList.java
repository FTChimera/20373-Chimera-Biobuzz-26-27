package org.firstinspires.ftc.teamcode.library.planners;


import static com.pedropathing.ivy.groups.Groups.sequential;

import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.ivy.commands.Commands;
import com.pedropathing.ivy.groups.Groups;


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

    public Command schedule() {
        Command command = asCommand();
        command.schedule();
        return command;
    }
    public Command asCommand() {
        List<Command> commands = new ArrayList<>();
        PathPlanner prevPlanner = new Utils.NullPlanner();
        for (PathPlanner pathPlanner : this) {
            PathPlanner finalPrevPlanner = prevPlanner;
            Command fromPrev = Commands.instant(() ->pathPlanner.fromPrev(finalPrevPlanner));
            prevPlanner = pathPlanner;
            commands.add(fromPrev);
            commands.add(pathPlanner.asCommand());
        }
        return Groups.sequential(commands.toArray(new Command[0]));
    }
}
