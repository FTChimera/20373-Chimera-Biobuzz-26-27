package org.firstinspires.ftc.teamcode.util.hardware.actuator;

import androidx.annotation.NonNull;

import com.pedropathing.revhub.drivetrains.CachedMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.util.Direction;
import org.firstinspires.ftc.teamcode.util.control.Controller;
import org.firstinspires.ftc.teamcode.util.control.DoubleFlywheelFullVelocityController;

import java.util.function.DoubleUnaryOperator;

public class SmartFlywheel {
    private static final double MIN_VALID_COMPENSATION_RATIO = 0.65;
    private static final double MAX_VALID_COMPENSATION_RATIO = 1.20;

    public CachedMotor motor;
    public double targetTicksPerSecond, targetPower;
    public boolean runningToVelocity;
    public Controller<Double, Double, Double> velocityController;
    public Params params;

    public SmartFlywheel(@NonNull DcMotorEx dcMotorEx, @NonNull Controller<Double, Double, Double> velocityController, @NonNull Params params) {
        this(dcMotorEx, velocityController, params, null);
    }

    public SmartFlywheel(@NonNull DcMotorEx dcMotorEx,
                         @NonNull Controller<Double, Double, Double> velocityController,
                         @NonNull Params params,
                         Iterable<VoltageSensor> voltageSensors) {
        motor = new CachedMotor(dcMotorEx, 0.005);
        this.velocityController = velocityController;
        this.params = params;
        setUpMotor();
        attachVoltageCompensationIfSupported(voltageSensors);
    }

    private void setUpMotor() {
        motor.raw().setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
    }

    public void setPower(double power) {
        if (!Double.isFinite(power) || Math.abs(power) > 1.0) {
            throw new IllegalArgumentException("Flywheel power must be finite and in [-1, 1].");
        }
        targetPower = power;
        runningToVelocity = false;
    }

    public void setVelocity(double velocity) {
        if (!Double.isFinite(velocity) || velocity < 0.0) {
            throw new IllegalArgumentException("Flywheel velocity must be finite and nonnegative ticks per second.");
        }
        targetTicksPerSecond = velocity;
        runningToVelocity = true;
        velocityController.setTarget(targetTicksPerSecond);
    }

    public void setRPM(double rpm) {
        if (!Double.isFinite(rpm) || rpm < 0.0) {
            throw new IllegalArgumentException("Flywheel RPM must be finite and nonnegative.");
        }
        targetTicksPerSecond = params.convertToTPS(rpm);
        runningToVelocity = true;
        velocityController.setTarget(targetTicksPerSecond);
    }

    public void stopImmediately() {
        targetPower = 0.0;
        targetTicksPerSecond = 0.0;
        runningToVelocity = false;
        velocityController.reset();
        motor.setPower(0);
    }

    public void update() {
        if (runningToVelocity) {
            velocityController.update(motor.raw().getVelocity());
            motor.setPower(velocityController.getOutput());
        } else {motor.setPower(targetPower);}
    }

    public Direction getDirection() {
        return motor.raw().getDirection() == com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD
                ? Direction.FORWARD : Direction.REVERSE;
    }

    public double getTicksPerSecond() {return motor.raw().getVelocity();}
    public double getRPM() {return params.convertToRPM(getTicksPerSecond());}

    private void attachVoltageCompensationIfSupported(Iterable<VoltageSensor> voltageSensors) {
        if (!(velocityController instanceof DoubleFlywheelFullVelocityController)) {
            return;
        }
        DoubleFlywheelFullVelocityController flywheelController =
                (DoubleFlywheelFullVelocityController) velocityController;
        flywheelController.setVoltageSupplier(() ->
                selectCompensationVoltage(
                        voltageSensors,
                        flywheelController.getConstants().nominalVoltage
                )
        );
    }

    static double selectCompensationVoltage(Iterable<VoltageSensor> voltageSensors, double nominalVoltage) {
        if (!Double.isFinite(nominalVoltage) || nominalVoltage <= 0.0) {
            throw new IllegalArgumentException("Nominal voltage must be finite and > 0.");
        }
        if (voltageSensors == null) {
            return Double.NaN;
        }
        double minVoltage = Double.NaN;
        double minValidVoltage = nominalVoltage * MIN_VALID_COMPENSATION_RATIO;
        double maxValidVoltage = nominalVoltage * MAX_VALID_COMPENSATION_RATIO;
        for (VoltageSensor sensor : voltageSensors) {
            if (sensor == null) {
                continue;
            }
            double voltage = sensor.getVoltage();
            if (!Double.isFinite(voltage) || voltage < minValidVoltage || voltage > maxValidVoltage) {
                continue;
            }
            if (Double.isNaN(minVoltage) || voltage < minVoltage) {
                minVoltage = voltage;
            }
        }
        return minVoltage;
    }

    public static class Params {
        public double encoderCpm, gearRatio;
        public Params(double encoderCpm, double gearRatio) {
            if (!Double.isFinite(encoderCpm) || encoderCpm <= 0.0
                    || !Double.isFinite(gearRatio) || gearRatio <= 0.0) {
                throw new IllegalArgumentException("Encoder counts-per-motor-rotation and gear ratio must be finite and > 0.");
            }
            this.encoderCpm = encoderCpm;
            this.gearRatio = gearRatio;
        }
        public double convertToRPM(double ticksPerSec) {return convertTPStoRPM().applyAsDouble(ticksPerSec);}
        public double convertToTPS(double rotationsPerMinute) {return convertRPMtoTPS().applyAsDouble(rotationsPerMinute);}
        public DoubleUnaryOperator convertTPStoRPM() {return tps -> tps * 60/ (encoderCpm*gearRatio);}
        public DoubleUnaryOperator convertRPMtoTPS() {return rpm -> rpm*encoderCpm*gearRatio/60;}
    }
}
