package org.firstinspires.ftc.teamcode.library.hardware.hardwareDevice;

import com.qualcomm.robotcore.hardware.AnalogInputController;
import com.qualcomm.robotcore.hardware.ControlSystem;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.configuration.annotations.AnalogSensorType;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import org.firstinspires.ftc.robotcore.external.ExportToBlocks;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * Proof of concept of custom hardware devices.
 * callable with hardwareMap.get(RevPotentiometer.class, "name")
  */
@AnalogSensorType
@DeviceProperties(
        name = "REV Potentiometer",
        xmlTag = "RevPotentiometer",
        compatibleControlSystems = {ControlSystem.REV_HUB}
)
public class RevPotentiometer implements HardwareDevice {
    private final AnalogInputController controller;
    private final int port;

    public RevPotentiometer(AnalogInputController controller, int port) {
        this.controller = controller;
        this.port = port;
    }

    @ExportToBlocks(
            tooltip = "Returns the current rotation of this potentiometer.",
            parameterLabels = {"angleUnit"}
    )
    public double getRotation(AngleUnit angleUnit) {
        return angleUnit.getUnnormalized().fromDegrees((double)270.0F * this.controller.getAnalogInputVoltage(this.port) / this.controller.getMaxAnalogInputVoltage());
    }

    public HardwareDevice.Manufacturer getManufacturer() {
        return Manufacturer.Lynx;
    }

    public String getDeviceName() {
        return "REV Potentiometer";
    }

    public String getConnectionInfo() {
        return this.controller.getConnectionInfo() + "; analog port " + this.port;
    }

    public int getVersion() {
        return 1;
    }

    public void resetDeviceConfigurationForOpMode() {
    }

    public void close() {
    }
}