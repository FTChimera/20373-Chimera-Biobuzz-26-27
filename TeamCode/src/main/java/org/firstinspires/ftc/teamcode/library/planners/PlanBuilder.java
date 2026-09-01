package org.firstinspires.ftc.teamcode.library.planners;

import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.Command;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.library.planners.paths.Utils;
import org.firstinspires.ftc.teamcode.mechanisms.Robot;
import org.firstinspires.ftc.teamcode.mechanisms.Vision;

import java.util.Objects;

public class PlanBuilder {
    PathPlannerList list;
    Robot robot;
    Vision vision;


    public PlanBuilder(Robot robot, Pose startPose) {
        this.robot = robot;
        this.vision = robot.getVision();
        list = new PathPlannerList();
        list.add(0, new Utils.StartPose(robot, startPose));
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
    public PlanBuilder intakeWVision(int numBalls, PathChain failSafePath) {
        if (vision == null) {
            throw new IllegalStateException(
                    "A Vision mechanism must be supplied through the constructor or withVision() before intakeWVision()."
            );
        }
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
