package org.firstinspires.ftc.teamcode.library.hardware.hardwareDevice;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.MotorControlAlgorithm;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.MotorType;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.teamcode.library.hardware.actuators.Motor;

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
        validateLimits(min, max);
        setMode(RunMode.STOP_AND_RESET_ENCODER);
        setMode(RunMode.RUN_WITHOUT_ENCODER);
        MAX_EXTENSION = max;
        MIN_RETRACTION = min;
    }

    @Override
    public void setPower(double power) {
        setMode(RunMode.RUN_WITHOUT_ENCODER);
        super.setPower(power);
    }
    public void setPosition(int position, double MaxPower) {
        validatePosition(position);
        setTargetPosition(position);
        setMode(RunMode.RUN_TO_POSITION);
        super.setPower(MaxPower);
    }
    public void setPosition(int position) {
        setPosition(position, Math.signum(position - getCurrentPosition()));
    }

    public void setPositionPercentage(double percentage) {
        validatePercentage(percentage);
        int position = (int) (MIN_RETRACTION + (MAX_EXTENSION - MIN_RETRACTION) * percentage);
        setPosition(position);
    }

    public void setPositionPercentage(double percentage, double MaxPower) {
        validatePercentage(percentage);
        int position = (int) (MIN_RETRACTION + (MAX_EXTENSION - MIN_RETRACTION) * percentage);
        setPosition(position, MaxPower);
    }
    public double getPositionPercentage() {
        validateLimits(MIN_RETRACTION, MAX_EXTENSION);
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

    private static void validateLimits(int min, int max) {
        if (min >= max) {
            throw new IllegalArgumentException("min retraction must be less than max extension.");
        }
    }

    private void validatePosition(int position) {
        validateLimits(MIN_RETRACTION, MAX_EXTENSION);
        if (position < MIN_RETRACTION || position > MAX_EXTENSION) {
            throw new IllegalArgumentException("position must be within the configured slide limits.");
        }
    }

    private static void validatePercentage(double percentage) {
        if (!Double.isFinite(percentage) || percentage < 0 || percentage > 1) {
            throw new IllegalArgumentException("percentage must be finite and between 0 and 1.");
        }
    }
}
