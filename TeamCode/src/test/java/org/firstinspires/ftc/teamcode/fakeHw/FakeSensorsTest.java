package org.firstinspires.ftc.teamcode.fakeHw;

import static org.junit.Assert.*;

import com.qualcomm.robotcore.hardware.*;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.navigation.*;
import org.junit.Test;

public class FakeSensorsTest {
    private static final double EPS = 1e-5;

    @Test public void voltageAndAnalogInputUseExplicitFiniteStimuli() {
        FakeVoltageSensor voltage = new FakeVoltageSensor(13.1);
        VoltageSensor sdkVoltage = voltage;
        assertEquals(13.1, sdkVoltage.getVoltage(), EPS);
        voltage.setVoltage(0);
        assertEquals(0, voltage.getVoltage(), 0);
        assertThrows(IllegalArgumentException.class, () -> voltage.setVoltage(-1));
        assertThrows(IllegalArgumentException.class, () -> voltage.setVoltage(Double.NaN));
        assertThrows(IllegalArgumentException.class, () -> voltage.setVoltage(Double.POSITIVE_INFINITY));

        FakeAnalogInput analog = new FakeAnalogInput(2, 3.3);
        AnalogInput sdkAnalog = analog;
        analog.setVoltage(2.7);
        assertEquals(2.7, sdkAnalog.getVoltage(), EPS);
        assertEquals(3.3, sdkAnalog.getMaxVoltage(), EPS);
        assertEquals("FakeAnalogInput", sdkAnalog.getDeviceName());
        assertTrue(sdkAnalog.getConnectionInfo().contains("analog port 2"));
        AnalogInput secondWrapper = new AnalogInput(analog.getController(), 2);
        assertEquals(2.7, secondWrapper.getVoltage(), EPS);
        assertThrows(IllegalArgumentException.class, () -> analog.getController().getAnalogInputVoltage(1));
        assertThrows(IllegalArgumentException.class, () -> analog.setVoltage(3.4));
        assertThrows(IllegalArgumentException.class, () -> analog.setVoltage(Double.NEGATIVE_INFINITY));
        assertThrows(UnsupportedOperationException.class, () -> analog.getController().getSerialNumber());
        sdkAnalog.resetDeviceConfigurationForOpMode();
        assertEquals(1, analog.getController().getResetCount());
        assertEquals(2.7, secondWrapper.getVoltage(), EPS);
        sdkAnalog.close();
        assertThrows(IllegalStateException.class, secondWrapper::getVoltage);
    }

    @Test public void digitalInputAndOutputAreSeparateAndHistoriesAreSnapshots() {
        FakeDigitalChannel channel = new FakeDigitalChannel();
        DigitalChannel sdk = channel;
        assertEquals(DigitalChannel.Mode.INPUT, sdk.getMode());
        assertTrue(sdk.getState());
        channel.setInputState(false);
        assertFalse(sdk.getState());
        assertThrows(IllegalStateException.class, () -> sdk.setState(true));
        assertEquals(0, channel.getStateWriteCount());
        sdk.setMode(DigitalChannelController.Mode.OUTPUT);
        sdk.setState(true);
        List<Boolean> before = channel.getStateHistory();
        sdk.setState(false);
        assertFalse(sdk.getState());
        assertEquals(1, before.size());
        assertEquals(2, channel.getStateWriteCount());
        assertThrows(UnsupportedOperationException.class, () -> before.add(false));
        sdk.resetDeviceConfigurationForOpMode();
        assertEquals(DigitalChannel.Mode.INPUT, sdk.getMode());
        assertFalse(sdk.getState());
        assertEquals(1, channel.getModeWriteCount());
        assertThrows(NullPointerException.class, () -> sdk.setMode((DigitalChannel.Mode) null));
        sdk.close();
        assertThrows(IllegalStateException.class, sdk::getState);
    }

    @Test public void distanceUnitsAndOutOfRangeAreExplicit() {
        FakeDistanceSensor sensor = new FakeDistanceSensor(12, DistanceUnit.INCH);
        DistanceSensor sdk = sensor;
        assertEquals(0.3048, sdk.getDistance(DistanceUnit.METER), EPS);
        assertEquals(30.48, sdk.getDistance(DistanceUnit.CM), EPS);
        assertEquals(304.8, sdk.getDistance(DistanceUnit.MM), EPS);
        sensor.setOutOfRange(true);
        for (DistanceUnit unit : DistanceUnit.values()) {
            assertEquals(DistanceSensor.distanceOutOfRange, sdk.getDistance(unit), 0);
        }
        sensor.setDistance(0, DistanceUnit.METER);
        assertEquals(0, sdk.getDistance(DistanceUnit.INCH), 0);
        assertThrows(IllegalArgumentException.class, () -> sensor.setDistance(-1, DistanceUnit.CM));
        assertThrows(IllegalArgumentException.class, () -> sensor.setDistance(Double.NaN, DistanceUnit.METER));
        assertThrows(IllegalArgumentException.class, () -> sensor.setDistance(Double.POSITIVE_INFINITY, DistanceUnit.METER));
        assertThrows(NullPointerException.class, () -> sdk.getDistance(null));
    }

    @Test public void colorImplementsAllThreeInterfacesAndCopiesReadings() {
        FakeColorSensor sensor = new FakeColorSensor();
        ColorSensor raw = sensor;
        NormalizedColorSensor normalized = sensor;
        DistanceSensor distance = sensor;
        sensor.setColor(255, 128, 0, 64);
        assertEquals(255, raw.red());
        assertEquals(128, raw.green());
        assertEquals(0, raw.blue());
        assertEquals(64, raw.alpha());
        assertEquals(0x40ff8000, raw.argb());
        assertEquals(0x40ff8000, normalized.getNormalizedColors().toColor());
        assertEquals(128 / 255f, normalized.getNormalizedColors().green, EPS);
        NormalizedRGBA old = normalized.getNormalizedColors();
        old.red = 0;
        assertEquals(1, normalized.getNormalizedColors().red, 0);
        sensor.setNormalizedColors(0.2f, 0.4f, 0.6f, 0.8f);
        normalized.setGain(2);
        assertEquals(0.4, normalized.getNormalizedColors().red, EPS);
        assertEquals(1, normalized.getNormalizedColors().blue, 0);
        assertEquals(0xff66ccff, normalized.getNormalizedColors().toColor());
        assertEquals(51, raw.red());
        sensor.setDistance(7, DistanceUnit.CM);
        assertEquals(70, distance.getDistance(DistanceUnit.MM), EPS);
        raw.enableLed(true);
        assertTrue(sensor.isLedEnabled());
        assertThrows(IllegalStateException.class, raw::getI2cAddress);
        I2cAddr address = I2cAddr.create7bit(0x29);
        raw.setI2cAddress(address);
        assertEquals(0x29, raw.getI2cAddress().get7Bit());
        assertThrows(NullPointerException.class, () -> raw.setI2cAddress(null));
        assertEquals(1, sensor.getLedWriteCount());
        assertEquals(1, sensor.getGainWriteCount());
        assertEquals(1, sensor.getAddressWriteCount());
        assertThrows(UnsupportedOperationException.class, () -> sensor.getGainHistory().clear());
        assertThrows(IllegalArgumentException.class, () -> sensor.setColor(256, 0, 0, 0));
        assertThrows(IllegalArgumentException.class, () -> sensor.setNormalizedColors(Float.NaN, 0, 0, 0));
        assertThrows(IllegalArgumentException.class, () -> sensor.setNormalizedColors(0, -0.1f, 0, 0));
        assertThrows(IllegalArgumentException.class, () -> normalized.setGain(Float.POSITIVE_INFINITY));
        assertThrows(IllegalArgumentException.class, () -> normalized.setGain(-1));
        sensor.resetDeviceConfigurationForOpMode();
        assertEquals(1, normalized.getGain(), 0);
        assertFalse(sensor.isLedEnabled());
        sensor.close();
        assertThrows(IllegalStateException.class, normalized::getNormalizedColors);
    }

    private static IMU.Parameters parameters() {
        return new IMU.Parameters(new ImuOrientationOnRobot() {
            @Override public Quaternion imuCoordinateSystemOrientationFromPerspectiveOfRobot() { return Quaternion.identityQuaternion(); }
            @Override public Quaternion imuRotationOffset() { return Quaternion.identityQuaternion(); }
            @Override public Quaternion angularVelocityTransform() { return Quaternion.identityQuaternion(); }
        });
    }

    @Test public void imuInitializationIsExplicitAndCopiesParameterContainer() {
        FakeIMU imu = new FakeIMU();
        assertFalse(imu.isInitialized());
        assertThrows(IllegalStateException.class, imu::getParameters);
        assertThrows(NullPointerException.class, () -> imu.initialize(null));
        assertThrows(NullPointerException.class, () -> imu.initialize(new IMU.Parameters(null)));
        IMU.Parameters parameters = parameters();
        assertTrue(imu.initialize(parameters));
        assertTrue(imu.isInitialized());
        assertEquals(1, imu.getInitializeCount());
        parameters.imuOrientationOnRobot = null;
        assertNotNull(imu.getParameters().imuOrientationOnRobot);
        IMU.Parameters copy = imu.getParameters();
        copy.imuOrientationOnRobot = null;
        assertNotNull(imu.getParameters().imuOrientationOnRobot);
    }

    @Test public void imuYawPitchRollQuaternionAndEulerAgreeWithFtcZxy() {
        FakeIMU imu = new FakeIMU();
        IMU sdk = imu;
        imu.setOrientation(AngleUnit.DEGREES, 40, 20, 10, 123);
        YawPitchRollAngles ypr = sdk.getRobotYawPitchRollAngles();
        assertEquals(40, ypr.getYaw(AngleUnit.DEGREES), EPS);
        assertEquals(20, ypr.getPitch(AngleUnit.DEGREES), EPS);
        assertEquals(10, ypr.getRoll(AngleUnit.DEGREES), EPS);
        assertEquals(123, ypr.getAcquisitionTime());
        Orientation euler = sdk.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.DEGREES);
        assertEquals(40, euler.firstAngle, EPS);
        assertEquals(20, euler.secondAngle, EPS);
        assertEquals(10, euler.thirdAngle, EPS);
        assertEquals(123, euler.acquisitionTime);
        Quaternion quaternion = sdk.getRobotOrientationAsQuaternion();
        assertEquals(1, quaternion.magnitude(), EPS);
        assertEquals(123, quaternion.acquisitionTime);
        quaternion.w = 0;
        assertNotEquals(0, sdk.getRobotOrientationAsQuaternion().w, 0);
        sdk.resetYaw();
        assertEquals(0, sdk.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES), EPS);
        assertEquals(20, sdk.getRobotYawPitchRollAngles().getPitch(AngleUnit.DEGREES), EPS);
        assertEquals(10, sdk.getRobotYawPitchRollAngles().getRoll(AngleUnit.DEGREES), EPS);
        imu.setOrientation(AngleUnit.DEGREES, 55, 20, 10, 456);
        assertEquals(15, sdk.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES), EPS);
        assertEquals(15, sdk.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.DEGREES).firstAngle, EPS);
        assertEquals(1, imu.getResetYawCount());
        imu.resetDeviceConfigurationForOpMode();
        assertEquals(15, sdk.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES), EPS);
    }

    @Test public void imuRatesAreUnnormalizedFiniteAndDefensivelyCopied() {
        FakeIMU imu = new FakeIMU();
        imu.setAngularVelocity(AngleUnit.DEGREES, 720, -360, 90, 99);
        AngularVelocity degrees = imu.getRobotAngularVelocity(AngleUnit.DEGREES);
        assertEquals(720, degrees.xRotationRate, 1e-4);
        assertEquals(-360, degrees.yRotationRate, 1e-4);
        assertEquals(90, degrees.zRotationRate, EPS);
        assertEquals(99, degrees.acquisitionTime);
        degrees.xRotationRate = 0;
        assertEquals(4 * Math.PI, imu.getRobotAngularVelocity(AngleUnit.RADIANS).xRotationRate, EPS);
        assertThrows(IllegalArgumentException.class, () -> imu.setAngularVelocity(AngleUnit.RADIANS, Double.MAX_VALUE, 0, 0, 0));
        assertThrows(IllegalArgumentException.class, () -> imu.setOrientation(AngleUnit.DEGREES, Double.NaN, 0, 0, 0));
        assertThrows(IllegalArgumentException.class, () -> imu.setOrientation(AngleUnit.RADIANS, 0, 0, 0, -1));
        imu.setOrientation(AngleUnit.DEGREES, 170, 0, 0, 0);
        imu.resetYaw();
        imu.setOrientation(AngleUnit.DEGREES, -170, 0, 0, 0);
        assertEquals(20, imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES), EPS);
        imu.close();
        assertThrows(IllegalStateException.class, imu::resetYaw);
    }

    @Test public void imuSupportsAllSdkEulerOrdersOnPlainJvm() {
        FakeIMU imu = new FakeIMU();
        imu.setOrientation(AngleUnit.DEGREES, 35, 25, 15, 17);
        for (AxesReference reference : AxesReference.values()) {
            for (AxesOrder order : AxesOrder.values()) {
                Orientation value = imu.getRobotOrientation(reference, order, AngleUnit.RADIANS);
                assertEquals(reference, value.axesReference);
                assertEquals(order, value.axesOrder);
                assertEquals(17, value.acquisitionTime);
                assertTrue(Float.isFinite(value.firstAngle));
                assertTrue(Float.isFinite(value.secondAngle));
                assertTrue(Float.isFinite(value.thirdAngle));
            }
        }

    }

    @Test public void imuSingularAndVeryLargeFiniteAnglesStayFinite() {
        FakeIMU imu = new FakeIMU();
        imu.setOrientation(AngleUnit.RADIANS, Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE, 0);
        assertTrue(Double.isFinite(imu.getRobotYawPitchRollAngles().getYaw()));
        for (int pitch = -90; pitch <= 90; pitch += 90) {
            imu.setOrientation(AngleUnit.DEGREES, 0, pitch, 0, 0);
            for (AxesReference reference : AxesReference.values()) {
                for (AxesOrder order : AxesOrder.values()) {
                    Orientation orientation = imu.getRobotOrientation(reference, order, AngleUnit.DEGREES);
                    assertTrue(Float.isFinite(orientation.firstAngle));
                    assertTrue(Float.isFinite(orientation.secondAngle));
                    assertTrue(Float.isFinite(orientation.thirdAngle));
                }
            }
        }
    }
}
