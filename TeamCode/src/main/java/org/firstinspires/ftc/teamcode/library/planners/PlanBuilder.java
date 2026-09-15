package org.firstinspires.ftc.teamcode.library.planners;

import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.Command;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.library.planners.paths.FromStartGardenPose;
import org.firstinspires.ftc.teamcode.library.planners.paths.Utils;
import org.firstinspires.ftc.teamcode.mechanisms.Robot;

public class PlanBuilder {
    public static PlanBuilder startGardenPose(Robot robot) {
        return new PlanBuilder(robot, Positions.GARDEN);
    }

    public static PlanBuilder nullPlan(Robot robot) {
        return new PlanBuilder(robot, Positions.NULL);
    }
    enum Positions {GARDEN, LOADING_ZONE, NULL, GARDEN_VISION_SEARCH, LOADING_VISION_SEARCH}
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
        if (startPosition==Positions.GARDEN) {
            return new FromStartGardenPose(robot);
        } else if (startPosition==Positions.LOADING_ZONE) {
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
    // add(new MyPlan(robot, lastPlanner(), ...));

    public PlanBuilder followPath(PathChain path) {
        add(new Utils.CustomPath(robot, planner().last(), path));
        return this;
    }
    public PlanBuilder linearPathTo(Pose endPose) {
        add(new Utils.LinearPath(robot, planner().last(), endPose));
        return this;
    }
    public PlanBuilder waitMs(double ms) {
        add(new Utils.Wait(planner().last(), ms));
        return this;
    }
    @Deprecated
    public PlanBuilder intakeWVision(int numBalls, PathChain failSafePath) {
        add(new Utils.IntakeWVision(robot, planner().last(), numBalls, failSafePath));
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
