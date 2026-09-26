package org.firstinspires.ftc.teamcode.library.game;


import org.firstinspires.ftc.teamcode.library.AllianceColor;
import org.firstinspires.ftc.teamcode.library.OpModeHelper;
import org.firstinspires.ftc.teamcode.library.Tuple;
import org.firstinspires.ftc.teamcode.library.hardware.gamepad.GpBindings;
import org.firstinspires.ftc.teamcode.mechanisms.Robot;

import java.util.Objects;

public class State {
    private GpBindings gpBindings;
    private OpModeHelper opModeHelper;
    private Robot robot;
    private Tuple<AllianceColor, RobotMode> allianceColorAndRobotMode;

    public void setAllianceColor(AllianceColor allianceColor) {
        allianceColorAndRobotMode = new Tuple<>(allianceColor, getRobotMode());
    }

    public AllianceColor getAllianceColor() {
        return allianceColorAndRobotMode.getA();
    }

    public Tuple<AllianceColor, RobotMode> getAllianceColorAndRobotMode() {
        return new Tuple<>(allianceColorAndRobotMode.getA(), allianceColorAndRobotMode.getB());
    }

    public void setAllianceColorAndRobotMode(Tuple<AllianceColor, RobotMode> allianceColorAndRobotMode) {
        Objects.requireNonNull(allianceColorAndRobotMode, "allianceColorAndRobotMode");
        RobotMode robotMode = allianceColorAndRobotMode.getB();
        if (robotMode == null) {
            throw new IllegalArgumentException("robotMode must not be null.");
        }
        this.allianceColorAndRobotMode = new Tuple<>(allianceColorAndRobotMode.getA(), robotMode);
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
        this(bindings, new Tuple<>(null, robotMode));
    }

    public State(Tuple<AllianceColor, RobotMode> allianceColorAndRobotMode) {
        this(null, allianceColorAndRobotMode);
    }

    public State(GpBindings bindings, Tuple<AllianceColor, RobotMode> allianceColorAndRobotMode) {
        setGpBindings(bindings);
        setAllianceColorAndRobotMode(allianceColorAndRobotMode);
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
        return allianceColorAndRobotMode.getB();
    }

    public void setRobotMode(RobotMode robotMode) {
        if (robotMode == null) {
            throw new IllegalArgumentException("robotMode must not be null.");
        }
        allianceColorAndRobotMode = new Tuple<>(
                allianceColorAndRobotMode.getA(),
                robotMode
        );
    }

    public State() {
        this(RobotMode.TELEOP);
    }
}
