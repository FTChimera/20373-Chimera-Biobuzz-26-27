package org.firstinspires.ftc.teamcode.library.game;

/** Identifies whether a robot control loop is running match autonomous or teleop behavior. */
public enum RobotMode {
    AUTONOMOUS(30),
    TELEOP(120);


    double limit;
    RobotMode(double timelimit) {
        limit = timelimit;
    }

    public double timeLimit() {
        return limit;
    }
}
