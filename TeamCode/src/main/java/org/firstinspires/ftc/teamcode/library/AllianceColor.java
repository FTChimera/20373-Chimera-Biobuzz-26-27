package org.firstinspires.ftc.teamcode.library;


/** A utility alliance color class that is used to track the color the robot is competing in in Autonomous and TeleOp*/
public enum AllianceColor {
    RED, BLUE;
    public AllianceColor opposite() {
        return this == RED ? BLUE : RED;
    }
}
