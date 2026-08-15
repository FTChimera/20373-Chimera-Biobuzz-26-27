package com.rmsftc.library.planners;

import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import com.rmsftc.library.planners.paths.Utils;
import org.firstinspires.ftc.teamcode.mechanisms.Mechanisms;

public class PlanBuilder {
    PathPlannerList list;
    Mechanisms mechanisms;
    public PlanBuilder(Mechanisms mechanisms, Pose startPose) {
        this.mechanisms = mechanisms;
        list = new PathPlannerList();
        list.add(0, new Utils.StartPose(mechanisms, startPose));
    }
    public PlanBuilder add(PathPlanner pathPlanner) {
        list.add(pathPlanner);
        return this;
    }

    // Here we add methods like
    // public PlanBuilder my-plan()
    // add(new MyPlan(mechanisms, lastPlanner(), ...));

    public PlanBuilder followPath(PathChain path) {
        add(new Utils.CustomPath(mechanisms, planner().last(), path));
        return this;
    }
    public PlanBuilder linearPathTo(Pose endPose) {
        add(new Utils.LinearPath(mechanisms, planner().last(), endPose));
        return this;
    }
    public PlanBuilder waitMs(double ms) {
        add(new Utils.Wait(planner().last(), ms));
        return this;
    }
    public PlanBuilder intakeWVision(int numBalls, PathChain failSafePath) {
        add(new Utils.IntakeWVision(mechanisms, planner().last(), numBalls, failSafePath));
        return this;
    }

    public PathPlannerList planner() {
        return list;
    }

    /** Build the Path Planner */
    public PathPlannerList build() {
        return list;
    }
}
