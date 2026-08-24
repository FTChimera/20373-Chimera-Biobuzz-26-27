package org.firstinspires.ftc.teamcode.library.planners;

import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.library.planners.paths.Utils;
import org.firstinspires.ftc.teamcode.mechanisms.Mechanisms;
import org.firstinspires.ftc.teamcode.mechanisms.Vision;

import java.util.Objects;

public class PlanBuilder {
    PathPlannerList list;
    Mechanisms mechanisms;
    Vision vision;

    public PlanBuilder(Mechanisms mechanisms, Pose startPose) {
        this(mechanisms, null, startPose);
    }

    public PlanBuilder(Mechanisms mechanisms, Vision vision, Pose startPose) {
        this.mechanisms = mechanisms;
        this.vision = vision;
        list = new PathPlannerList();
        list.add(0, new Utils.StartPose(mechanisms, startPose));
    }

    public PlanBuilder withVision(Vision vision) {
        this.vision = Objects.requireNonNull(vision, "vision");
        return this;
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
        if (vision == null) {
            throw new IllegalStateException(
                    "A Vision mechanism must be supplied through the constructor or withVision() before intakeWVision()."
            );
        }
        add(new Utils.IntakeWVision(mechanisms, vision, planner().last(), numBalls, failSafePath));
        return this;
    }

    public PathPlannerList planner() {
        return list;
    }

    /** Build the Path Planner */
    public PathPlannerList build() {
        return list;
    }

    public void schedule() {
        list.schedule();
    }
}
