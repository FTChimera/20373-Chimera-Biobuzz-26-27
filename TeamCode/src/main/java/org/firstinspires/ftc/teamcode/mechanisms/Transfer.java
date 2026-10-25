package org.firstinspires.ftc.teamcode.mechanisms;


import com.pedropathing.revhub.drivetrains.CachedMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.Color;
import org.firstinspires.ftc.teamcode.util.Timer;
import org.firstinspires.ftc.teamcode.util.game.Ball;
import org.firstinspires.ftc.teamcode.util.game.BallCol;
import org.firstinspires.ftc.teamcode.util.Direction;
import org.firstinspires.ftc.teamcode.util.hardware.sensors.ColorSensor;

import java.util.InvalidPropertiesFormatException;
import java.util.concurrent.TimeUnit;

public class Transfer {
    private static final double NECTAR_KICK_TIME = 500; // in milliseconds
    private static final double POLLEN_KICK_TIME = 300; // TODO: tune these values

    public enum Mode {
        KICK_ONE,
        SHOOT_ALL,
        SMART_SHOOT,
        SLOW,
        CUSTOM_POWER,
        PANIC_SHOOT,
        IDLE,
        REVERSE_EJECT,
        BLOCK
    }

    private final CachedMotor transferMotor;
//    private final ColorSensor colorSensor;
// TODO: 10/24/26 uncomment this
    private boolean modeSwitch = false;
    private Mode mode = Mode.IDLE;
    Timer modeTimer;

    public Transfer(HardwareMap hardwareMap) {
        transferMotor = new CachedMotor(hardwareMap.get(DcMotorEx.class, "transfer"), 0.01);
        transferMotor.setDirection(Direction.FORWARD.toMotor());
//        colorSensor = new ColorSensor(hardwareMap, "transferCol", Color.TRANSPARENT, 4F, ColorSensor.COST_THRESHOLD_DEFAULT);
    }

    public Mode mode() {
        return mode;
    }

    public boolean isModeSwitch() {
        return modeSwitch;
    }
    BallCol prevBallColor;
    Mode prevMode;
    public void kickOne() {
        mode = Mode.KICK_ONE;
        modeSwitch = true;
    }

    public void smartShoot() {
        mode = Mode.SMART_SHOOT;
        modeSwitch = true;
    }

    public BallCol getBallColor() {
//        try {
            return BallCol.POLLEN_YELLOW;
//            return colorSensor.getBallCol();
//        } catch (InvalidPropertiesFormatException e) {
//            return BallCol.UNKNOWN;
//        }
    }

    public void slow() {
        mode = Mode.SLOW;
        modeSwitch = true;
    }
    double customPower;
    public void customPower(double power) {
        if (!(Math.abs(power) <= 1.0 && !Double.isNaN(power))) throw new IllegalArgumentException("Power must be between -1.0 and 1.0");
        customPower = power;
        mode = Mode.CUSTOM_POWER;
        modeSwitch = true;
    }

    public void setMode(Mode newMode) {
        this.mode = newMode;
        modeSwitch = true;
    }

    public void shootAll() {
        mode = Mode.SHOOT_ALL;
        modeSwitch = true;
    }

    public void panicShoot() {
        mode = Mode.PANIC_SHOOT;
        modeSwitch = true;
    }

    public void reverse() {
        mode = Mode.REVERSE_EJECT;
        modeSwitch = true;
    }

    public void block() {
        mode = Mode.BLOCK;
        modeSwitch = true;
    }

    public void update() {
//        colorSensor.update();
        if (mode != Mode.KICK_ONE) prevMode = Mode.values()[mode.ordinal()];
        switch (mode) {
            case IDLE:
                if (modeSwitch) transferMotor.setPower(0);
                break;
            case KICK_ONE:
                if (modeSwitch) {
                    modeTimer = new Timer(TimeUnit.MILLISECONDS);
                    prevBallColor = getBallColor();
                    transferMotor.setPower(0.5);
                }
                if (getBallColor() != prevBallColor || modeTimer.past(prevBallColor.ball== Ball.NECTAR? NECTAR_KICK_TIME : POLLEN_KICK_TIME)) {
                    mode = Mode.values()[prevMode.ordinal()];
                    modeSwitch = true;
                    return;
                }
                break;
            case SHOOT_ALL:
                transferMotor.setPower(0.8);
                break;
            case PANIC_SHOOT:
                transferMotor.setPower(1);
                break;
            case SLOW:
                transferMotor.setPower(0.4);
                break;
            case REVERSE_EJECT:
                transferMotor.setPower(-0.8);
                break;
            case BLOCK:
                transferMotor.setPower(-0.05);
                break;
            case CUSTOM_POWER:
                transferMotor.setPower(customPower);
                break;
            case SMART_SHOOT:
                // TODO: Not implemented
                transferMotor.setPower(0);
                break;
        }
        modeSwitch = false;
    }

    public void stop() {
        mode = Mode.IDLE;
        modeSwitch = true;
    }

    public void stopImmediately() {
        mode = Mode.IDLE;
        modeSwitch = true;
        transferMotor.setPower(0);
    }


    public String getDebugString() {
        return "Mode: " + mode + "\n Detected Ball: " + getBallColor();
    }
}
