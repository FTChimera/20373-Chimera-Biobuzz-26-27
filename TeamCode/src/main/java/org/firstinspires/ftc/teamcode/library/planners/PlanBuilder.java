package org.firstinspires.ftc.teamcode.library.planners;

import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.Command;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.library.planners.paths.FromShootGardenPose;
import org.firstinspires.ftc.teamcode.library.planners.paths.FromStartGardenPose;
import org.firstinspires.ftc.teamcode.library.planners.paths.Utils;
import org.firstinspires.ftc.teamcode.mechanisms.Robot;

public class PlanBuilder {
    public static PlanBuilder startGardenPose(Robot robot) {
        return new PlanBuilder(robot, Positions.GARDEN_START);
    }

    public static PlanBuilder nullPlan(Robot robot) {
        return new PlanBuilder(robot, Positions.NULL);
    }
    enum Positions {GARDEN_START, LOADING_START, GARDEN_SHOOT, LOADING_SHOOT, NULL, GARDEN_VISION_SEARCH, LOADING_VISION_SEARCH}
    PathPlannerList list;
    Robot robot;
    Positions position;

    private PlanBuilder(Robot robot, Positions startPosition) {
        this.robot = robot;
        list = new PathPlannerList();
        position = startPosition;
        list.add(0, getPlannerFromPositionStart(startPosition, robot));
    }

    private PathPlanner getPlannerFromPositionStart(Positions startPosition, Robot robot) {
        if (startPosition==Positions.GARDEN_START) {
            return new FromStartGardenPose(robot);
        } else if (startPosition==Positions.LOADING_START) {
            return null;
        }
        return new Utils.StartPose(robot, Constants.RED_DEFAULT_START_POSE);
    }

    public PlanBuilder add(PathPlanner pathPlanner) {
        list.add(pathPlanner);
        return this;
    }

    // Here we add methods like
    // public PlanBuilder my-plan()
    // add(new MyPlan(robot, ...));

    public PlanBuilder shootPreloads() {
        if (position == Positions.GARDEN_START) {
            list.add(new FromStartGardenPose.ShootPreloads(robot));
            position = Positions.GARDEN_SHOOT;
        } else if (position == Positions.LOADING_START) {
            // Todo implement module
            position = Positions.LOADING_SHOOT;
        } else {
            throw new IllegalStateException("Cannot shoot Preloads at position " + position);
        }
        return this;
    }

    public PlanBuilder shootPreloadsToVision() {
        if (position == Positions.GARDEN_START) {
            list.add(new FromStartGardenPose.ShootPreloadsToVisionSearch(robot));
            position = Positions.GARDEN_VISION_SEARCH;
        } else if (position == Positions.LOADING_START) {
            // Todo implement module
            position = Positions.LOADING_SHOOT;
        } else {
            throw new IllegalStateException("Cannot shoot Preloads at position " + position);
        }
        return this;
    }

    public PlanBuilder intakeGarden() {
        if (position == Positions.GARDEN_SHOOT) {
            list.add(new FromShootGardenPose.ToIntakeGardenPollen(robot));
        } // todo implement a loading to garden thing too
        else {
            throw new IllegalStateException("Cannot intake from the garden at position " + position);
        }
        return this;
    }

    public PlanBuilder intakeGardenToVision() {
        if (position == Positions.GARDEN_SHOOT) {
            list.add(new FromShootGardenPose.ToIntakeGardenPollenToVision(robot));
            position = Positions.GARDEN_VISION_SEARCH;
        }// todo implement a loading to garden thing too
        else {
            throw new IllegalStateException("Cannot intake from the garden at position " + position);
        }
        return this;
    }





    // These are the utility methods

    public PlanBuilder followPath(PathChain path) {
        add(new Utils.CustomPath(robot, path));
        return this;
    }
    public PlanBuilder linearPathTo(Pose endPose) {
        add(new Utils.LinearPath(robot, planner().last(), endPose));
        return this;
    }
    public PlanBuilder waitMs(double ms) {
        add(new Utils.Wait(ms));
        return this;
    }
    @Deprecated
    public PlanBuilder intakeWVision(int numBalls, PathChain failSafePath) {
        add(new Utils.IntakeWVision(robot, numBalls, failSafePath));
        return this;
    }

    public PathPlannerList planner() {
        return list;
    }

    /** Build the Path Planner */
    public PathPlannerList build() {
        return list;
    }

    public Command schedule() {
        return list.schedule();
    }
}
