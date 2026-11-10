package org.firstinspires.ftc.teamcode.mechanisms;



import com.pedropathing.revhub.drivetrains.CachedMotor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.AllianceColor;
import org.firstinspires.ftc.teamcode.util.Direction;

public class Intake {
    CachedMotor mainMotor;

    // Deployable ramp (deployer) servo positions. TODO: TUNE - positions not
    // yet set on the robot.
    public static final double DEPLOYER_DEPLOYED_POSITION = 1.0;
    public static final double DEPLOYER_STOWED_POSITION = 0.0;

    Servo deployer;
    private boolean deployed = false;

    AllianceColor allianceColor;

    public enum Mode {
        FULL_POWER,
        LOW_POWER,
        IDLE,
        REVERSE,
        EJECTING,
        CUSTOM_POWER;
    }

    private Mode mode = Mode.IDLE;
    double customPower=0;
    private boolean modeSwitch = false;
    public boolean allowOverrideOverEject = false;

    public Intake(HardwareMap map, AllianceColor color) {
        mainMotor = new CachedMotor(map.get(DcMotorEx.class, "intake"), 0.1);

        mainMotor.setDirection(Direction.FORWARD.toMotor());

        deployer = map.get(Servo.class, "deployer");
        setDeployed(false);
        internalSet(0);
        this.allianceColor = color;
    }
    public Mode mode() {
        return mode;
    }

    /** Sets the deployable ramp: true = deployed (ramp down), false = stowed. */
    public void setDeployed(boolean deployed) {
        this.deployed = deployed;
        deployer.setPosition(deployed ? DEPLOYER_DEPLOYED_POSITION : DEPLOYER_STOWED_POSITION);
    }

    public boolean isDeployed() {
        return deployed;
    }
    public void setFullPower() {
        if (isEjecting() && !allowOverrideOverEject) return;
        mode = Mode.FULL_POWER;
        modeSwitch=true;
    }

    public void setLowPower() {
        if (isEjecting() && !allowOverrideOverEject) return;
        mode = Mode.LOW_POWER;
        modeSwitch = true;
    }

    public void stop() {
        if (isEjecting() && !allowOverrideOverEject) return;
        mode = Mode.IDLE;
        modeSwitch=true;
    }

    public void stopImmediately() {
        internalSet(0);
        stop();
    }

    public void reverse() {
        if (isEjecting() && !allowOverrideOverEject) return;
        mode = Mode.REVERSE;
        modeSwitch=true;
    }

    public void setCustomPower(double power) {
        if (isEjecting() && !allowOverrideOverEject) return;
        if (!(Math.abs(power) <= 1.0 && !Double.isNaN(power))) throw new IllegalArgumentException("Power must be between -1.0 and 1.0");
        mode = Mode.CUSTOM_POWER;
        customPower = power;
        modeSwitch=true;
    }

    public void setMode(Mode mode) {
        if (isEjecting() && !allowOverrideOverEject) return;
        this.mode = mode;
        modeSwitch = true;
    }


    public boolean isEjecting() {return mode==Mode.EJECTING;}
    public Mode prev = null;
    public void forceEject() {
        if (isEjecting() && !allowOverrideOverEject) return;
        prev = mode;
        mode = Mode.EJECTING;
        modeSwitch = true;
    }
    public void stopEjecting() {
        if (prev == null || prev == Mode.EJECTING) {
            mode = Mode.IDLE;
        } else {
            mode = prev;
        }
        modeSwitch = true;
    }

    public void update() {//Twist twist) {
        if (mode != Mode.EJECTING) prev = Mode.values()[mode.ordinal()];
        if (prev == Mode.EJECTING) prev = Mode.IDLE;
        //double twistDir = twist.vy;
        switch (mode) {
            case IDLE:
                if (modeSwitch) internalSet(0);
                break;
            case FULL_POWER:
                //internalSet(twistDir>=0?1:0.8);
                internalSet(1);
                break;
            case LOW_POWER:
                //internalSet(twistDir>=0?0.6:0.4);
                internalSet(0.7);
                break;
            case REVERSE:
                internalSet(-0.5);
                break;
            case CUSTOM_POWER:
                internalSet(customPower);
                break;
            case EJECTING:
                internalSet(prev==Mode.LOW_POWER? -0.5: -0.7);
                break;
        }
        modeSwitch=false;
    }

    public boolean isModeSwitch() {
        return modeSwitch;
    }

    public void internalSet(double power) {
        mainMotor.setPower(power);
    }

    public String getDebugString() {
        return "Mode, " + mode.toString()
                + "\n Deployer: " + (deployed ? "DEPLOYED" : "STOWED");
    }
}
