package org.firstinspires.ftc.teamcode.library.hardware.hardwareDevice;

import com.qualcomm.robotcore.hardware.ControlSystem;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.ServoControllerEx;
import com.qualcomm.robotcore.hardware.configuration.ServoFlavor;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.ServoType;

@SuppressWarnings("SpellCheckingInspection")
/*
 * GOBILDA PRODUCT INSIGHT: https://cdn11.bigcommerce.com/s-x56mtydx1w/images/stencil/original/products/2275/15126/3118-0808-0002-Product-Insight-4__88285.1757516465.png?c=1
 */
@ServoType(flavor = ServoFlavor.CUSTOM)
@DeviceProperties(
        name = "GoBilda RGB Indicator",
        xmlTag = "RGBIndicator",
        compatibleControlSystems = {ControlSystem.REV_HUB})
public class RGBIndicator implements HardwareDevice {
    private ServoControllerEx controller;
    private int port;
    /**
     * Returns an indication of the manufacturer of this device.
     *
     * @return the device's manufacturer
     */
    @Override
    public Manufacturer getManufacturer() {
        return Manufacturer.GoBilda;
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
        return "GoBilda RGB Indicator";
    }

    /**
     * Get connection information about this device in a human readable format
     *
     * @return connection info
     */
    @Override
    public String getConnectionInfo() {
        return "null";
    }

    /**
     * Version
     *
     * @return get the version of this device
     */
    @Override
    public int getVersion() {
        return 1;
    }

    /**
     * Resets the device's configuration to that which is expected at the beginning of an OpMode.
     * For example, motors will reset the their direction to 'forward'.
     */
    @Override
    public void resetDeviceConfigurationForOpMode() {
        setColor(Color.BLACK);
    }

    /**
     * Closes this device
     */
    @Override
    public void close() {
        setColor(Color.BLACK);
    }
    public enum Color {
        BLACK(BLACK_PWM),
        RED(RED_PWM),
        ORANGE(ORANGE_PWM),
        GOLD(GOLD_PWM),
        YELLOW(YELLOW_PWM),
        SAGE(SAGE_PWM),
        GREEN(GREEN_PWM),
        AZURE(AZURE_PWM),
        BLUE(BLUE_PWM),
        INDIGO(INDIGO_PWM),
        VIOLET(VIOLET_PWM),
        WHITE(WHITE_PWM),
        UNKNOWN(BLACK_PWM);
        private final double pwmValue;
        Color(double pwmValue) {
            this.pwmValue = pwmValue;
        }
        public double getPWM() {
            return pwmValue;
        }
    }
    /*
    Black – 0
    Red – 0.279
    Orange – 0.333
    Gold – 0.357
    Yellow – 0.388
    Sage – 0.444
    Green – 0.500
    Azure – 0.555
    Blue – 0.611
    Indigo – 0.666
    Violet – 0.722
    White – 1
    */
    public static double BLACK_PWM = 0.0;
    public static double RED_PWM = 0.279;
    public static double ORANGE_PWM = 0.333;
    public static double GOLD_PWM = 0.357;
    public static double YELLOW_PWM = 0.388;
    public static double SAGE_PWM = 0.444;
    public static double GREEN_PWM = 0.500;
    public static double AZURE_PWM = 0.555;
    public static double BLUE_PWM = 0.611;
    public static double INDIGO_PWM = 0.666;
    public static double VIOLET_PWM = 0.722;
    public static double WHITE_PWM = 1.0;
    public RGBIndicator(ServoControllerEx controller, int port) {
        this.controller = controller;
        this.port = port;
    }
    public void setCustomPWM(double val) {
        internalSetPwm(val);
    }
    public void setColor(Color col) {
        internalSetPwm(col.getPWM());
    }
    public double getPWM() {
        return controller.getServoPosition(port);
    }
    public Color getColor() {
        for (Color color : Color.values()) {
            if (Math.abs(getPWM() - color.getPWM()) < 0.01) {
                return color;
            }
        }
        return Color.UNKNOWN;
    }
    private void internalSetPwm(double pwm) {
        controller.setServoPosition(port, pwm);
    }
}