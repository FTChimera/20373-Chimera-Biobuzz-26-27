package org.firstinspires.ftc.teamcode.util.hardware.actuator;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import java.util.Arrays;

import org.junit.Test;

public class SmartFlywheelParamsTest {
    private static final double EPSILON = 1e-9;

    @Test
    public void convertsTicksPerSecondToRpmAndBack() {
        SmartFlywheel.Params params = new SmartFlywheel.Params(28.0, 1.0);

        double rpm = params.convertToRPM(1400.0);
        double ticksPerSecond = params.convertToTPS(rpm);

        assertEquals(3000.0, rpm, EPSILON);
        assertEquals(1400.0, ticksPerSecond, EPSILON);
    }

    @Test(expected = IllegalArgumentException.class)
    public void rejectsNonPositiveCountsPerMotorRotation() {
        new SmartFlywheel.Params(0.0, 1.0);
    }

    @Test
    public void compensationVoltageUsesLowestValidReadingWithinBounds() {
        double selected = SmartFlywheel.selectCompensationVoltage(
                Arrays.asList(
                        new FakeVoltageSensor(Double.NaN),
                        new FakeVoltageSensor(14.6),
                        new FakeVoltageSensor(12.8),
                        new FakeVoltageSensor(11.9)
                ),
                12.0
        );

        assertEquals(11.9, selected, EPSILON);
    }

    @Test
    public void compensationVoltageReturnsNanWhenNoSensorReadingIsValid() {
        double selected = SmartFlywheel.selectCompensationVoltage(
                Arrays.asList(
                        new FakeVoltageSensor(8.0),
                        new FakeVoltageSensor(0.0),
                        new FakeVoltageSensor(Double.POSITIVE_INFINITY)
                ),
                12.0
        );

        assertTrue(Double.isNaN(selected));
    }

    private static final class FakeVoltageSensor implements VoltageSensor {
        private final double voltage;

        private FakeVoltageSensor(double voltage) {
            this.voltage = voltage;
        }

        @Override
        public double getVoltage() {
            return voltage;
        }

        @Override
        public HardwareDevice.Manufacturer getManufacturer() {
            return HardwareDevice.Manufacturer.Other;
        }

        @Override
        public String getDeviceName() {
            return "fake";
        }

        @Override
        public String getConnectionInfo() {
            return "test";
        }

        @Override
        public int getVersion() {
            return 1;
        }

        @Override
        public void resetDeviceConfigurationForOpMode() {
        }

        @Override
        public void close() {
        }
    }
}
