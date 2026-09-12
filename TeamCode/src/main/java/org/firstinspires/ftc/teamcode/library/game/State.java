package org.firstinspires.ftc.teamcode.library.game;


import org.firstinspires.ftc.teamcode.library.AllianceColor;
import org.firstinspires.ftc.teamcode.library.OpModeHelper;
import org.firstinspires.ftc.teamcode.library.hardware.gamepad.GpBindings;
import org.firstinspires.ftc.teamcode.mechanisms.Robot;

public class State {
    public HiveState state;
    public GpBindings gpBindings;
    public OpModeHelper opModeHelper;
    public Robot robot;
    public AllianceColor allianceColor;
    private RobotMode robotMode;

    public void setAllianceColor(AllianceColor allianceColor) {
        this.allianceColor = allianceColor;
    }

    public AllianceColor getAllianceColor() {
        return allianceColor;
    }

    public void setRobot(Robot robot) {
        this.robot = robot;
    }

    public Robot getRobot() {
        return robot;
    }

    public void setOpModeHelper(OpModeHelper opModeHelper) {
        this.opModeHelper = opModeHelper;
    }

    public OpModeHelper getOpModeHelper() {
        return opModeHelper;
    }
    public State(GpBindings bindings) {
        this(bindings, RobotMode.TELEOP);
    }

    public State(RobotMode robotMode) {
        this(null, robotMode);
    }

    public State(GpBindings bindings, RobotMode robotMode) {
        setGpBindings(bindings);
        setRobotMode(robotMode);
    }
    public GpBindings getGpBindings() {
        return gpBindings;
    }

    public void setGpBindings(GpBindings gpBindings) {
        this.gpBindings = gpBindings;
    }

    public State(GpBindings b, OpModeHelper helper, Robot robot) {
        this(b);
        this.opModeHelper = helper;
        this.robot = robot;
    }

    public RobotMode getRobotMode() {
        return robotMode;
    }

    public void setRobotMode(RobotMode robotMode) {
        if (robotMode == null) {
            throw new IllegalArgumentException("robotMode must not be null.");
        }
        this.robotMode = robotMode;
    }

    public State() {
        this(RobotMode.TELEOP);
    }
}
