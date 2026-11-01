package org.firstinspires.ftc.teamcode.planners;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.groups.Groups;

import org.firstinspires.ftc.teamcode.mechanisms.Robot;
import org.firstinspires.ftc.teamcode.pedro.commands.Scheduler;
import org.firstinspires.ftc.teamcode.planners.modules.FromShootGardenPose;
import org.firstinspires.ftc.teamcode.planners.modules.FromStartGardenPose;
import org.firstinspires.ftc.teamcode.planners.modules.FromStartLoadingPose;
import org.firstinspires.ftc.teamcode.planners.modules.Utils;
import org.firstinspires.ftc.teamcode.util.poses.Pose2d;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public class PlanBuilder {

    enum Positions {GARDEN_START, LOADING_START, GARDEN_SHOOT, LOADING_SHOOT, NULL, GARDEN_VISION_SEARCH, LOADING_VISION_SEARCH}

    public static PlanBuilder startGarden(Robot robot) {
        return new PlanBuilder(robot, Positions.GARDEN_START);
    }

    public static PlanBuilder startLoading(Robot robot) {
        return new PlanBuilder(robot, Positions.LOADING_START);
    }

    Positions position;
    volatile Robot robot;
    ArrayList<PathPlanner> planners;

    private PlanBuilder(Robot robot, Positions startPosition) {
        this.robot = robot;
        position = startPosition;
        planners = new ArrayList<>(Collections.singletonList(new Utils.NullPlanner(
                getStartPositionPose2d(startPosition)
        )));
    }

    public PlanBuilder waitMs(double ms) {
        planners.add(new Utils.WaitMs(robot, ms));
        return this;
    }

    public PlanBuilder shootPreloads() {
        if (position==Positions.GARDEN_START) {
            planners.add(new FromStartGardenPose.ShootPreloads(robot));
            position=Positions.GARDEN_SHOOT;
        } else if (position==Positions.LOADING_START) {
            planners.add(new FromStartLoadingPose.ShootPreloads(robot));
            position=Positions.GARDEN_SHOOT;
        } else {
            throw new IllegalStateException("Cannot Shoot Preloads from Position " + position);
        }
        return this;
    }

    public PlanBuilder intakeFromGarden() {
        if (position==Positions.GARDEN_SHOOT) {
            planners.add(new FromShootGardenPose.ToIntakeGarden(robot));
        } else if (position==Positions.LOADING_START) {
            position=Positions.GARDEN_SHOOT;
        } else {
            throw new IllegalStateException("Cannot Intake from the Garden from Position " + position);
        }
        return this;
    }

    private Pose2d getStartPositionPose2d(Positions startPosition) {
        if (startPosition==Positions.GARDEN_START) {
            return StartPoses.gardenArea;
        }
        if (startPosition==Positions.LOADING_START) {
            return StartPoses.loadingArea;
        }
        throw new IllegalArgumentException("Position argument not a start pose");
    }
    public PathPlanner getFirst() {
        return planners.get(0);
    }
    public PathPlanner getLast() {
        return planners.get(planners.size()-1);
    }

    public ArrayList<PathPlanner> getList() {
        return planners;
    }

    public Command schedule() {
        return schedule(Scheduler.INSTANCE);
    }

    public Command schedule(Scheduler scheduler) {
        Command command = asCommand();
        scheduler.schedule(command);
        return command;
    }
    public Command asCommand() {
        List<Command> commands = new ArrayList<>();
        PathPlanner prevPlanner = getFirst();
        for (PathPlanner pathPlanner : planners) {
            PathPlanner finalPrevPlanner = prevPlanner;
            prevPlanner = pathPlanner;
            commands.add(pathPlanner.total(finalPrevPlanner.getEndPose()));
        }
        return Groups.sequential(commands.toArray(new Command[0]));
    }
}
