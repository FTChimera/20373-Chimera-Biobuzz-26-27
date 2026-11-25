package org.firstinspires.ftc.teamcode.fakeHw;

import static org.junit.Assert.*;

import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.hardware.configuration.ServoFlavor;
import com.qualcomm.robotcore.hardware.configuration.annotations.ServoType;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.ServoConfigurationType;
import java.util.List;
import org.junit.Test;

public class FakeServosTest {
    private static final double EPS = 1e-9;
    @ServoType(flavor = ServoFlavor.STANDARD)
    private static final class StandardServo { }

    @Test public void servoClipsScalesReversesAndKeepsImmutableCommandHistory() {
        FakeServo fake = new FakeServo();
        Servo servo = fake;
        servo.scaleRange(0.2, 0.8);
        servo.setDirection(Servo.Direction.REVERSE);
        servo.setPosition(0.25);
        assertEquals(0.65, fake.getPhysicalPosition(), EPS);
        assertEquals(0.25, servo.getPosition(), EPS);
        List<Double> history = fake.getPositionHistory();
        servo.setPosition(2);
        assertEquals(0.2, fake.getPhysicalPosition(), EPS);
        assertEquals(1, servo.getPosition(), EPS);
        servo.setPosition(-2);
        assertEquals(0.8, fake.getPhysicalPosition(), EPS);
        assertEquals(0, servo.getPosition(), EPS);
        assertEquals(1, history.size());
        assertEquals(3, fake.getPositionWriteCount());
        assertEquals(3, fake.getController().getPositionWriteCount());
        assertThrows(UnsupportedOperationException.class, () -> history.add(0.5));
        assertThrows(IllegalArgumentException.class, () -> servo.setPosition(Double.NaN));
        assertThrows(IllegalArgumentException.class, () -> servo.setPosition(Double.POSITIVE_INFINITY));
        assertThrows(IllegalArgumentException.class, () -> servo.scaleRange(0.8, 0.2));
        assertThrows(IllegalArgumentException.class, () -> servo.scaleRange(0.5, 0.5));
        assertThrows(IllegalArgumentException.class, () -> servo.scaleRange(Double.NaN, 1));
        assertEquals(3, fake.getPositionWriteCount());
        servo.resetDeviceConfigurationForOpMode();
        assertEquals(Servo.Direction.FORWARD, servo.getDirection());
        assertEquals(0.8, servo.getPosition(), EPS);
        servo.setPosition(0.1);
        assertEquals(0.1, fake.getPhysicalPosition(), EPS);
        FakeServoController controller = fake.getController();
        servo.close();
        assertThrows(IllegalStateException.class, () -> servo.setPosition(0));
        assertFalse(controller.isClosed());
    }

    @Test public void crServoPowerMatchesSdkControllerMapping() {
        FakeCRServo fake = new FakeCRServo();
        CRServo servo = fake;
        assertEquals(0, servo.getPower(), EPS);
        servo.setDirection(DcMotorSimple.Direction.REVERSE);
        servo.setPower(0.4);
        assertEquals(0.4, servo.getPower(), EPS);
        assertEquals(-0.4, fake.getPhysicalPower(), EPS);
        assertEquals(0.3, fake.getController().getServoPosition(0), EPS);
        List<Double> history = fake.getPowerHistory();
        servo.setPower(4);
        assertEquals(1, servo.getPower(), EPS);
        assertEquals(-1, fake.getPhysicalPower(), EPS);
        servo.setPower(-4);
        assertEquals(-1, servo.getPower(), EPS);
        assertEquals(1, fake.getPhysicalPower(), EPS);
        assertEquals(1, history.size());
        assertEquals(3, fake.getPowerWriteCount());
        assertThrows(UnsupportedOperationException.class, history::clear);
        assertThrows(IllegalArgumentException.class, () -> servo.setPower(Double.NEGATIVE_INFINITY));
        assertThrows(NullPointerException.class, () -> servo.setDirection(null));
        servo.resetDeviceConfigurationForOpMode();
        assertEquals(DcMotorSimple.Direction.FORWARD, servo.getDirection());
        assertEquals(1, servo.getPower(), EPS);
    }

    @Test public void realSdkServoAndCrServoWrappersUseControllerState() {
        FakeServoController controller = new FakeServoController(2);
        Servo servo = new ServoImpl(controller, 0);
        CRServo cr = new CRServoImpl(controller, 1);
        servo.scaleRange(0.1, 0.7);
        servo.setDirection(Servo.Direction.REVERSE);
        servo.setPosition(0.2);
        assertEquals(0.58, controller.getServoPosition(0), EPS);
        assertEquals(0.2, servo.getPosition(), EPS);
        assertEquals(2, controller.getForgetCount(0));
        cr.setDirection(DcMotorSimple.Direction.REVERSE);
        cr.setPower(0.6);
        assertEquals(0.2, controller.getServoPosition(1), EPS);
        assertEquals(0.6, cr.getPower(), EPS);
        assertEquals(2, controller.getPositionWriteCount());
        assertThrows(IllegalArgumentException.class, () -> servo.setPosition(Double.NaN));
        assertThrows(IllegalArgumentException.class, () -> cr.setPower(Double.NaN));
    }

    @Test public void pwmControlsHaveIndependentPortsAndPulseConversion() {
        FakeServoController controller = new FakeServoController(2);
        FakeServo servo = new FakeServo(controller, 0);
        servo.setPwmRange(new PwmControl.PwmRange(500, 2500, 20000));
        servo.setPwmDisable();
        assertFalse(servo.isPwmEnabled());
        assertEquals(ServoController.PwmStatus.MIXED, controller.getPwmStatus());
        controller.pwmDisable();
        assertEquals(ServoController.PwmStatus.DISABLED, controller.getPwmStatus());
        servo.setPulseWidth(1500);
        assertEquals(0.5, servo.getPhysicalPosition(), EPS);
        assertEquals(1500, servo.getPulseWidth(), EPS);
        assertTrue(servo.isPwmEnabled());
        servo.setPulseWidth(99999);
        assertEquals(2500, servo.getPulseWidth(), EPS);
        controller.pwmEnable();
        assertEquals(ServoController.PwmStatus.ENABLED, controller.getPwmStatus());
        List<FakeServoController.PositionWrite> history = controller.getPositionHistory();
        assertEquals(0, history.get(0).port);
        assertEquals(0.5, history.get(0).position, EPS);
        assertThrows(UnsupportedOperationException.class, history::clear);
        assertThrows(IllegalArgumentException.class, () -> controller.setServoPosition(2, 0));
        assertThrows(IllegalArgumentException.class, () -> controller.getServoPosition(-1));
        assertThrows(IllegalArgumentException.class, () -> servo.setPulseWidth(Double.NaN));
        assertThrows(IllegalArgumentException.class, () -> servo.setPwmRange(new PwmControl.PwmRange(2000, 1000)));
        assertThrows(IllegalArgumentException.class, () -> servo.setPwmRange(new PwmControl.PwmRange(500, 2500, Double.NaN)));
        assertThrows(IllegalArgumentException.class, () -> servo.setPwmRange(new PwmControl.PwmRange(500, 2500, 2000)));
        controller.resetDeviceConfigurationForOpMode();
        assertEquals(PwmControl.PwmRange.defaultRange, servo.getPwmRange());
        controller.close();
        assertThrows(IllegalStateException.class, () -> controller.getServoPosition(0));
    }

    @Test public void realSdkServoImplExUsesExtendedController() {
        FakeServoController controller = new FakeServoController(1);
        ServoConfigurationType type = new ServoConfigurationType();
        type.processAnnotation(StandardServo.class.getAnnotation(ServoType.class));
        ServoImplEx servo = new ServoImplEx(controller, 0, type);
        servo.setPwmRange(new PwmControl.PwmRange(700, 2300));
        assertEquals(700, servo.getPwmRange().usPulseLower, EPS);
        servo.setPwmDisable();
        assertFalse(servo.isPwmEnabled());
        servo.setPwmEnable();
        assertTrue(servo.isPwmEnabled());
        servo.setPosition(0.25);
        assertEquals(1100, servo.getPulseWidth(), EPS);
        servo.setPulseWidth(1900);
        assertEquals(0.75, servo.getPosition(), EPS);
    }
}
