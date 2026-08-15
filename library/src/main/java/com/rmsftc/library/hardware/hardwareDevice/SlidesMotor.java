package com.rmsftc.library.hardware.hardwareDevice;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.MotorControlAlgorithm;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.MotorType;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import com.rmsftc.library.hardware.actuators.Motor;

@MotorType(ticksPerRev = 560, gearing = 20, maxRPM = 300) // for ultraplanetary system
@DeviceProperties(
        name = "Slides Motor",
        xmlTag = "SlidesMotor",
        compatibleControlSystems = {com.qualcomm.robotcore.hardware.ControlSystem.REV_HUB})
public class SlidesMotor extends DcMotorImplEx {
    public static final String DEFAULT_HARDWAREMAP_NAME = "Slides";
    public int MAX_EXTENSION = 3500;
    public int MIN_RETRACTION = 0;

    public SlidesMotor(DcMotorController controller, int portNumber) {
        super(controller, portNumber);
    }

    public SlidesMotor(DcMotorController controller, int portNumber, Direction direction) {
        super(controller, portNumber, direction);
    }

    public SlidesMotor(DcMotorController controller, int portNumber, Direction direction, @NonNull MotorConfigurationType motorType) {
        super(controller, portNumber, direction, motorType);
    }
    public SlidesMotor(Motor motor) {
        super(motor.getMotor().getController(), motor.getMotor().getPortNumber(), motor.getMotor().getDirection(), motor.getMotor().getMotorType());
    }

    public void initialize(int min, int max) {
        setMode(RunMode.STOP_AND_RESET_ENCODER);
        setMode(RunMode.RUN_WITHOUT_ENCODER);
        //set min and max extension limits
        MAX_EXTENSION = max;
        MIN_RETRACTION = min;
    }

    @Override
    public void setPower(double power) {
        setMode(RunMode.RUN_WITHOUT_ENCODER);
        super.setPower(power);
    }
    public void setPosition(int position, double MaxPower) {
        setTargetPosition(position);
        setMode(RunMode.RUN_TO_POSITION);
        super.setPower(MaxPower);
    }
    public void setPosition(int position) {
        setPosition(position, Math.signum(position - getCurrentPosition()));
    }

    public void setPositionPercentage(double percentage) {
        int position = (int) (MIN_RETRACTION + (MAX_EXTENSION - MIN_RETRACTION) * percentage);
        setPosition(position);
    }

    public void setPositionPercentage(double percentage, double MaxPower) {
        int position = (int) (MIN_RETRACTION + (MAX_EXTENSION - MIN_RETRACTION) * percentage);
        setPosition(position, MaxPower);
    }
    public double getPositionPercentage() {
        return (double) (getCurrentPosition() - MIN_RETRACTION) / (MAX_EXTENSION - MIN_RETRACTION);
    }

    public void setUp() {
        setPosition(MAX_EXTENSION);
    }

    public void setDown() {
        setPosition(MIN_RETRACTION);
    }

    public void setUp(double MaxPower) {setPosition(MAX_EXTENSION, MaxPower);}
    public void setDown(double MaxPower) {setPosition(MIN_RETRACTION, MaxPower);}

    public void setPositionPIDFCoefficients(double p, double i, double d, double f) {
        setPIDFCoefficients(RunMode.RUN_TO_POSITION, new PIDFCoefficients(p, i, d, f, MotorControlAlgorithm.PIDF));
    }
}
