package org.firstinspires.ftc.teamcode.planners;


import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.groups.Groups;


import org.firstinspires.ftc.teamcode.planners.modules.Utils;
import org.firstinspires.ftc.teamcode.util.poses.Pose2d;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public class PathPlannerList extends ArrayList<PathPlanner> implements List<PathPlanner> {
    public PathPlannerList(Pose2d startPose) {
        super(Collections.singleton(new Utils.NullPlanner(startPose)));
    }

    /**
     * Sets the first path planner:
     * @param startPose The starting pose to set.
     */
    public void setStartPose(Pose2d startPose) {
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
            prevPlanner = pathPlanner;
            commands.add(pathPlanner.total(finalPrevPlanner.getEndPose()));
        }
        return Groups.sequential(commands.toArray(new Command[0]));
    }
}
