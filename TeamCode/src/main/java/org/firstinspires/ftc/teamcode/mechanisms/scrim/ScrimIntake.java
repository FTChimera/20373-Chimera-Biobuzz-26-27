package org.firstinspires.ftc.teamcode.mechanisms.scrim;

import com.pedropathing.math.Twist;
import com.pedropathing.revhub.drivetrains.CachedMotor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.util.AllianceColor;
import org.firstinspires.ftc.teamcode.util.Color;
import org.firstinspires.ftc.teamcode.util.Direction;
import org.firstinspires.ftc.teamcode.util.game.BallCol;
import org.firstinspires.ftc.teamcode.util.hardware.sensors.ColorSensor;

import java.util.InvalidPropertiesFormatException;

public class ScrimIntake {
    CachedMotor mainMotor;
    AllianceColor allianceColor;
    private Intake.Mode mode = Intake.Mode.IDLE;
    double customPower=0;
    private boolean modeSwitch = false;
    public boolean allowOverrideOverEject = false;

    public ScrimIntake(HardwareMap map, AllianceColor color) {
        mainMotor = new CachedMotor(map.get(DcMotorEx.class, "intake"), 0.1);

        mainMotor.setDirection(Direction.FORWARD.toMotor());

        internalSet(0);
        this.allianceColor = color;
    }
    public Intake.Mode mode() {
        return mode;
    }
    public void setFullPower() {
        if (isEjecting() && !allowOverrideOverEject) return;
        mode = Intake.Mode.FULL_POWER;
        modeSwitch=true;
    }

    public void setLowPower() {
        if (isEjecting() && !allowOverrideOverEject) return;
        mode = Intake.Mode.LOW_POWER;
        modeSwitch = true;
    }

    public void stop() {
        if (isEjecting() && !allowOverrideOverEject) return;
        mode = Intake.Mode.IDLE;
        modeSwitch=true;
    }

    public void stopImmediately() {
        internalSet(0);
        stop();
    }

    public void reverse() {
        if (isEjecting() && !allowOverrideOverEject) return;
        mode = Intake.Mode.REVERSE;
        modeSwitch=true;
    }

    public void setCustomPower(double power) {
        if (isEjecting() && !allowOverrideOverEject) return;
        if (!(Math.abs(power) <= 1.0 && !Double.isNaN(power))) throw new IllegalArgumentException("Power must be between -1.0 and 1.0");
        mode = Intake.Mode.CUSTOM_POWER;
        customPower = power;
        modeSwitch=true;
    }

    public void setMode(Intake.Mode mode) {
        if (isEjecting() && !allowOverrideOverEject) return;
        this.mode = mode;
        modeSwitch = true;
    }

    public boolean isEjecting() {return mode== Intake.Mode.EJECTING;}
    public Intake.Mode prev = null;
    public void forceEject() {
        if (isEjecting() && !allowOverrideOverEject) return;
        mode = Intake.Mode.EJECTING;
        modeSwitch = true;
    }

    public void stopEjecting() {
        mode = Intake.Mode.values()[prev.ordinal()];
    }

    public void update() {//Twist twist) {
        if (mode != Intake.Mode.EJECTING) prev = Intake.Mode.values()[mode.ordinal()];

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
                internalSet(prev== Intake.Mode.LOW_POWER? -0.5: -0.7);
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
       return BallCol.UNKNOWN;
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
