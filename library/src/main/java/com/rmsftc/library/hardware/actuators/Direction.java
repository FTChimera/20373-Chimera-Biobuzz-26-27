package com.rmsftc.library.hardware.actuators;

import com.qualcomm.robotcore.hardware.DcMotorSimple;

public enum Direction {
    FORWARD, REVERSE;

    public Direction reverse() {
        return this == FORWARD ? REVERSE : FORWARD;
    }
    public DcMotorSimple.Direction toFtcSdk() {
        return this == FORWARD ? DcMotorSimple.Direction.FORWARD : DcMotorSimple.Direction.REVERSE;
    }
}
