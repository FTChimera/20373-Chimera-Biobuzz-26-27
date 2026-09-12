package org.firstinspires.ftc.teamcode.library;

import org.firstinspires.ftc.robotcore.external.ExportEnumToBlocks;

/** A utility alliance color class that is used to track the color the robot is competing in Autonomous and TeleOp*/
@ExportEnumToBlocks(color = 124)
public enum AllianceColor {
    RED, BLUE;
    public boolean isRed() {return this==RED;}
    public AllianceColor opposite() {
        return this == RED ? BLUE : RED;
    }
}
