package org.firstinspires.ftc.teamcode.library.hardware.actuators;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

/**
 * Custom Motor class that wraps around the DcMotorEx class and provides additional functionality for easier motor control.
 * <pre>
 * Not callable with hardwareMap.get() for now
 *
 * TODO Add a constructor Motor(DcMotorControllerEx controller, int port) to allow hardware Map get from SDK
 * Then use the controller Ex instead of the DcMotorEx
 * Uncomment the below
 */

/*@MotorType(ticksPerRev = 28, gearing = 3, maxRPM = 2000) // random default values
@DeviceProperties(
        name = "Motor",
        xmlTag = "Motor",
        compatibleControlSystems = {ControlSystem.REV_HUB}
)*/
public class Motor implements HardwareDevice {
    double targetPower, targetVelocity;
    DcMotorEx motor;

    public Motor(HardwareMap hwMap, String name) {
        motor = hwMap.get(DcMotorEx.class, name);
    }
    public Motor reverse() {
        reverseDirection();
        return this;
    }
    public void reverseDirection() {
        motor.setDirection(motor.getDirection().inverted());
    }

    public Motor Float() {
        setFloat();
        return this;
    }

    public void setFloat() {
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    public Motor Brake() {
        setBrake();
        return this;
    }

    public void setBrake() {
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public Motor PIDFControl(PIDFCoefficients coefficients) {
        setPIDFCoefficients(coefficients);
        return this;
    }
    public Motor PIDFControl(double P, double I, double D, double F) {
        return PIDFControl(new PIDFCoefficients(P, I, D, F));
    }

    public void setPIDFCoefficients(double P, double I, double D, double F) {
        setPIDFCoefficients(new PIDFCoefficients(P, I, D, F));
    }
    public void setPIDFCoefficients(PIDFCoefficients coefficients) {
        motor.setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER,
                coefficients
        );
    }

    public DcMotorEx getMotor() {
        return motor;
    }


    public void setPower(double power) {
        targetPower = power;
        motor.setPower(targetPower);
    }

    public double getPower() {
        return motor.getPower();
    }
    public double getTargetPower() {
        return targetPower;
    }
    public double getTargetVelocity() {
        return targetVelocity;
    }
    public void stopVelocity() {
        motor.setVelocity(0);
        targetVelocity = 0;
    }
    public void setVelocity(double velocity) {
        targetVelocity = velocity;
        motor.setVelocity(targetVelocity);
    }
    public void stopPower() {
        motor.setPower(0);
        targetPower = 0;
    }

    public Motor setDirection(Direction motorDirection) {
        motor.setDirection(motorDirection.toFtcSdk());
        return this;
    }
    public Direction getDirection() {
        return motor.getDirection() == com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD
                ? Direction.FORWARD : Direction.REVERSE;
    }
    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        motor.setZeroPowerBehavior(zeroPowerBehavior);
    }

    /**
     * Returns an indication of the manufacturer of this device.
     *
     * @return the device's manufacturer
     */
    @Override
    public Manufacturer getManufacturer() {
        return Manufacturer.Other;
    }

    /**
     * Returns a string suitable for display to the user as to the type of device.
     * Note that this is a device-type-specific name; it has nothing to do with the
     * name by which a user might have configured the device in a robot configuration.
     *
     * @return device manufacturer and name
     */
    @Override
    public String getDeviceName() {
        return "Motor";
    }

    /**
     * Get connection information about this device in a human readable format
     *
     * @return connection info
     */
    @Override
    public String getConnectionInfo() {
        return motor.getConnectionInfo();
    }

    /**
     * Version
     *
     * @return get the version of this device
     */
    @Override
    public int getVersion() {
        return motor.getVersion();
    }

    /**
     * Resets the device's configuration to that which is expected at the beginning of an OpMode.
     * For example, motors will reset the their direction to 'forward'.
     */
    @Override
    public void resetDeviceConfigurationForOpMode() {
        motor.resetDeviceConfigurationForOpMode();
        setDirection(Direction.FORWARD);
    }

    /**
     * Closes this device
     */
    @Override
    public void close() {
        motor.close();
    }
}
