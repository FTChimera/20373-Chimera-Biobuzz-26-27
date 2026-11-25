package org.firstinspires.ftc.teamcode.mechanisms;


import com.pedropathing.revhub.drivetrains.CachedMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.Timer;
import org.firstinspires.ftc.teamcode.util.Direction;
import org.firstinspires.ftc.teamcode.planners.Constants;

import java.util.concurrent.TimeUnit;

public class Transfer {
    public enum Mode {
        KICK_ONE,
        SHOOT_ALL,
        SLOW,
        CUSTOM_POWER,
        PANIC_SHOOT,
        IDLE,
        REVERSE_EJECT,
        BLOCK
    }

    private final CachedMotor transferMotor;
    private boolean modeSwitch = false;
    private Mode mode = Mode.IDLE;
    Timer modeTimer;

    public Transfer(HardwareMap hardwareMap) {
        transferMotor = new CachedMotor(hardwareMap.get(DcMotorEx.class, "transfer"), Constants.Transfer.CACHE_INTERVAL_S);
        transferMotor.setDirection(Direction.FORWARD.toMotor());
    }

    public Mode mode() {
        return mode;
    }

    public boolean isModeSwitch() {
        return modeSwitch;
    }
    // Initialized to IDLE: kickOne() restores prevMode when the kick ends,
    // and a null prevMode would NPE there if kickOne() ran before the
    // first update().
    Mode prevMode = Mode.IDLE;
    public void kickOne() {
        mode = Mode.KICK_ONE;
        modeSwitch = true;
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
        if (mode != Mode.KICK_ONE) prevMode = Mode.values()[mode.ordinal()];
        switch (mode) {
            case IDLE:
                if (modeSwitch) transferMotor.setPower(0);
                break;
            case KICK_ONE:
                if (modeSwitch) {
                    modeTimer = new Timer(TimeUnit.MILLISECONDS);
                    transferMotor.setPower(Constants.Transfer.KICK_ONE_POWER);
                }
                if (modeTimer.past(Constants.Transfer.POLLEN_KICK_TIME_MS)) {
                    mode = Mode.values()[prevMode.ordinal()];
                    modeSwitch = true;
                    return;
                }
                break;
            case SHOOT_ALL:
                transferMotor.setPower(Constants.Transfer.SHOOT_ALL_POWER); //decreased for accuracy
                break;
            case PANIC_SHOOT:
                transferMotor.setPower(Constants.Transfer.PANIC_SHOOT_POWER);
                break;
            case SLOW:
                transferMotor.setPower(Constants.Transfer.SLOW_POWER);
                break;
            case REVERSE_EJECT:
                transferMotor.setPower(Constants.Transfer.REVERSE_EJECT_POWER);
                break;
            case BLOCK:
                transferMotor.setPower(Constants.Transfer.BLOCK_POWER);
                break;
            case CUSTOM_POWER:
                transferMotor.setPower(customPower);
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
        return "Mode: " + mode;
    }
}
