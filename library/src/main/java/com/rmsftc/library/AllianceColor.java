package com.rmsftc.library;


import org.firstinspires.ftc.robotcore.external.ExportEnumToBlocks;

/** A utility alliance color class that is used to track the color the robot is competing in in Autonomous and TeleOp*/
@ExportEnumToBlocks(color = 124)
public enum AllianceColor {
    RED, BLUE;
    public AllianceColor opposite() {
        return this == RED ? BLUE : RED;
    }
}
