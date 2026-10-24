package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

public enum Direction {
    FORWARD(Servo.Direction.FORWARD, DcMotorSimple.Direction.FORWARD),
    REVERSE(Servo.Direction.REVERSE, DcMotorSimple.Direction.REVERSE);
    private Servo.Direction ftcsdk_servo;
    private DcMotorSimple.Direction ftcsdk_motor;
    Direction(Servo.Direction direction, DcMotorSimple.Direction direction1) {
        ftcsdk_servo=direction;
        ftcsdk_motor = direction1;
    }

    public Servo.Direction toServo() {
        return ftcsdk_servo;
    }

    public DcMotorSimple.Direction toMotor() {
        return ftcsdk_motor;
    }

    public DcMotorSimple.Direction toCRServo() {
        return ftcsdk_motor;
    }

    public Direction inverted() {
        return this==FORWARD? REVERSE:FORWARD;
    }
}
