package org.firstinspires.ftc.teamcode.robotDev.subsystemTests;



import com.pedropathing.math.Twist;
import com.pedropathing.revhub.drivetrains.CachedMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.AllianceColor;
import org.firstinspires.ftc.teamcode.util.Color;
import org.firstinspires.ftc.teamcode.util.game.BallCol;
import org.firstinspires.ftc.teamcode.util.Direction;
import org.firstinspires.ftc.teamcode.util.hardware.sensors.ColorSensor;

import java.util.InvalidPropertiesFormatException;

/** An Intake without the CRServos and only 1 sensor */
public class IntakeTestPrototype {
    CachedMotor mainMotor;

    AllianceColor allianceColor;

    ColorSensor colorSensorRight;

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
    private boolean autoEject = false;
    public boolean allowOverrideOverEject = false;

    public IntakeTestPrototype(HardwareMap map, AllianceColor color) {
        mainMotor = new CachedMotor(map.get(DcMotorEx.class, "intake"), 0.1);

        mainMotor.setDirection(Direction.FORWARD.toMotor());

        colorSensorRight = new ColorSensor(map, "color_sensor", Color.GRAY, 65.3F, ColorSensor.COST_THRESHOLD_DEFAULT);
        internalSet(0);
        this.allianceColor = color;
    }
    public Mode mode() {
        return mode;
    }
    public void setFullPower() {
        if (isEjecting() && !allowOverrideOverEject) return;
        mode = Mode.FULL_POWER;
        modeSwitch=true;
    }

    public void setLowPower() {
        if (isEjecting() && !allowOverrideOverEject) return;
        prev = Mode.LOW_POWER;
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
        mode = Mode.CUSTOM_POWER;
        customPower = power;
        modeSwitch=true;
    }

    public void setMode(Mode mode) {
        if (isEjecting() && !allowOverrideOverEject) return;
        this.mode = mode;
        modeSwitch = true;
    }
    public void setAutoEject(boolean autoEject) {
        this.autoEject = autoEject;
    }

    public boolean isAutoEjectSet() {
        return autoEject;
    }

    public boolean isEjecting() {return mode== Mode.EJECTING;}
    public Mode prev = null;
    public void forceEject() {
        //prev = mode;
        mode = Mode.EJECTING;
        modeSwitch = true;
    }

    public void update() {//Twist twist) {
        colorSensorRight.update();
        if (isAutoEjectSet() && isOpposingAllianceNectar()) {
            forceEject();
        }
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
                internalSet(-1);
                break;
            case CUSTOM_POWER:
                internalSet(customPower);
                break;
            case EJECTING:
                internalSet(prev== Mode.LOW_POWER? -0.5: -0.7);
                if (isAutoEjectSet() && !isOpposingAllianceNectar()) {
                    setMode(prev);
                }
                break;
        }
        modeSwitch=false;
    }

    /** Determined if a robot will move significantly from its location; useful if an opposing alliance nectar is intook and ejected then if the velocity is high then we might not intake that same one*/
    public boolean determineMoved(Twist twist) {
        return twist.vy<-0.2 || Math.abs(twist.vx) > 0.6; // TODO: Deterimine these values and others. These are assuming clipped from 0-1 but actually are in in/s. Find robots max velocity and divide
    }

    public boolean isModeSwitch() {
        return modeSwitch;
    }

    public BallCol getCurrentCol() {
        try {
            return colorSensorRight.getBallCol();
        } catch (InvalidPropertiesFormatException e) {
            return BallCol.UNKNOWN;
        }
    }

    public boolean isOpposingAllianceNectar() {
        return getCurrentCol()==opposingAllianceNectar();
    }

    public BallCol opposingAllianceNectar() {
        return allianceColor.isRed() ? BallCol.NECTAR_BLUE: BallCol.NECTAR_RED;
    }

    public void internalSet(double power) {
        mainMotor.setPower(power);
    }

    public String getDebugString() {
        return "Mode, " + mode.toString() + "\n Detected Ball: " + getCurrentCol().toString();
    }
}
