package org.firstinspires.ftc.teamcode.library.game;


import org.firstinspires.ftc.teamcode.library.AllianceColor;
import org.firstinspires.ftc.teamcode.library.OpModeHelper;
import org.firstinspires.ftc.teamcode.library.hardware.gamepad.GpBindings;
import org.firstinspires.ftc.teamcode.mechanisms.RobotDrive;

public class State {
    public GpBindings gpBindings;
    public OpModeHelper opModeHelper;
    public RobotDrive robotDrive;
    public AllianceColor allianceColor;

    public void setAllianceColor(AllianceColor allianceColor) {
        this.allianceColor = allianceColor;
    }

    public AllianceColor getAllianceColor() {
        return allianceColor;
    }

    public void setRobotDrive(RobotDrive robotDrive) {
        this.robotDrive = robotDrive;
    }

    public RobotDrive getRobotDrive() {
        return robotDrive;
    }

    public void setOpModeHelper(OpModeHelper opModeHelper) {
        this.opModeHelper = opModeHelper;
    }

    public OpModeHelper getOpModeHelper() {
        return opModeHelper;
    }
    public State(GpBindings bindings) {
        setGpBindings(gpBindings);
    }
    public GpBindings getGpBindings() {
        return gpBindings;
    }

    public void setGpBindings(GpBindings gpBindings) {
        this.gpBindings = gpBindings;
    }

    public State(GpBindings b, OpModeHelper helper, RobotDrive robotDrive) {
        this.gpBindings = b;
        this.opModeHelper = helper;
        this.robotDrive = robotDrive;
    }
    public State() {

    }
}
