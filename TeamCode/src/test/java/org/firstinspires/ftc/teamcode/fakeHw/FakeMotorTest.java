package org.firstinspires.ftc.teamcode.fakeHw;

import static org.junit.Assert.*;

import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;
import com.qualcomm.robotcore.hardware.MotorControlAlgorithm;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Rotation;
import org.junit.Test;

public class FakeMotorTest {
    private static final double EPS = 1e-8;

    private FakeDcMotorEx plant() {
        FakeDcMotorEx motor = new FakeDcMotorEx();
        motor.setFreeSpeedTicksPerSecond(1000);
        motor.setTimeConstant(0.2);
        return motor;
    }

    @Test public void exactIntegrationIsPartitionInvariantAndVoltageScaled() {
        FakeDcMotorEx whole = plant();
        FakeDcMotorEx split = plant();
        whole.setPower(0.7); split.setPower(0.7);
        whole.advance(1);
        for (int i = 0; i < 100; i++) split.advance(0.01);
        assertEquals(700 * (1 - Math.exp(-5)), whole.getVelocity(), EPS);
        assertEquals(700 * (1 - 0.2 * (1 - Math.exp(-5))), whole.getExactPosition(), EPS);
        assertEquals(whole.getExactPosition(), split.getExactPosition(), EPS);
        assertEquals(whole.getVelocity(), split.getVelocity(), EPS);
        whole.setBatteryVoltage(6); split.setBatteryVoltage(6);
        whole.setPower(-0.4); split.setPower(-0.4);
        whole.advance(0.8);
        for (int i = 0; i < 80; i++) split.advance(0.01);
        assertEquals(whole.getExactPosition(), split.getExactPosition(), EPS);
        assertEquals(whole.getVelocity(), split.getVelocity(), EPS);
        FakeDcMotorEx low = plant();
        low.setBatteryVoltage(6); low.setPower(0.7); low.advance(1);
        assertEquals(700 * (1 - Math.exp(-5)) / 2, low.getVelocity(), EPS);
        assertEquals(1, split.getElapsedSeconds() - 0.8, EPS);
    }

    @Test public void clippingDirectionOrientationEncoderResetAndStimulus() {
        FakeDcMotorEx motor = plant();
        motor.setDirection(Direction.REVERSE);
        motor.setPower(4);
        assertEquals(1, motor.getPower(), EPS);
        assertEquals(-1, motor.getPhysicalPower(), EPS);
        motor.advance(1);
        assertTrue(motor.getVelocity() > 0);
        assertTrue(motor.getPhysicalPosition() < 0);
        motor.setEncoderPosition(100.25);
        motor.addEncoderTicks(3);
        assertEquals(103.25, motor.getExactPosition(), EPS);
        assertEquals(103, motor.getCurrentPosition());
        double shaft = motor.getPhysicalPosition();
        motor.setMode(RunMode.STOP_AND_RESET_ENCODER);
        assertEquals(0, motor.getCurrentPosition());
        assertEquals(0, motor.getVelocity(), EPS);
        motor.setPower(1); motor.advance(1);
        assertEquals(shaft, motor.getPhysicalPosition(), EPS);
        motor.setMode(RunMode.RUN_WITHOUT_ENCODER); motor.advance(0.5);
        assertTrue(motor.getCurrentPosition() > 0);
        motor.setDirection(Direction.FORWARD);
        assertTrue(motor.getCurrentPosition() < 0);
        MotorConfigurationType type = motor.getMotorType();
        type.setOrientation(Rotation.CCW);
        motor.setMotorType(type);
        assertTrue(motor.getCurrentPosition() > 0);
        motor.setPower(-5);
        assertEquals(-1, motor.getPower(), EPS);
    }

    @Test public void runToPositionUsesMagnitudeAndStopsInsideToleranceBothDirections() {
        FakeDcMotorEx motor = plant();
        motor.setTargetPositionTolerance(3);
        motor.setDirection(Direction.REVERSE);
        motor.setTargetPosition(100);
        motor.setMode(RunMode.RUN_TO_POSITION);
        motor.setPower(-0.5);
        assertEquals(0.5, motor.getPower(), EPS);
        assertTrue(motor.isBusy());
        motor.advance(10);
        assertFalse(motor.isBusy());
        assertEquals(97, motor.getCurrentPosition());
        assertEquals(0, motor.getVelocity(), EPS);
        motor.setTargetPosition(-100);
        motor.advance(10);
        assertEquals(-97, motor.getCurrentPosition());
        assertFalse(motor.isBusy());
        motor.setEncoderPosition(-96);
        assertTrue(motor.isBusy());
        motor.setPower(0);
        motor.advance(1);
        assertEquals(-96, motor.getCurrentPosition());
        motor.setTargetPositionTolerance(4);
        assertTrue(motor.isBusy());
        assertEquals(4, motor.getTargetPositionTolerance());
        motor.setTargetPosition(-100);
        assertFalse(motor.isBusy());
        assertThrows(IllegalArgumentException.class, () -> motor.setTargetPositionTolerance(-1));
    }

    @Test public void rawControllerTargetUsesDefaultToleranceNotPreviousTargetTolerance() {
        FakeDcMotorController controller = new FakeDcMotorController(1);
        FakeDcMotorEx motor = controller.getMotor(0);
        controller.setMotorMode(0, RunMode.RUN_TO_POSITION);
        controller.setMotorTargetPosition(0, 100, 20);
        motor.setEncoderPosition(90);
        assertFalse(controller.isBusy(0));
        controller.setMotorTargetPosition(0, 100);
        assertTrue(controller.isBusy(0));
        motor.setEncoderPosition(95);
        assertFalse(controller.isBusy(0));
    }

    @Test public void velocityUsesTicksPerRevolutionAndClipsToAvailableSpeed() {
        FakeDcMotorEx motor = plant();
        MotorConfigurationType type = motor.getMotorType();
        type.setTicksPerRev(720); type.setMaxRPM(100); type.setAchieveableMaxRPMFraction(1);
        motor.setMotorType(type);
        assertEquals(1200, motor.getFreeSpeedTicksPerSecond(), EPS);
        motor.setTimeConstant(0.01);
        motor.setVelocity(360, AngleUnit.DEGREES); motor.advance(1);
        assertEquals(720, motor.getVelocity(), EPS);
        assertEquals(360, motor.getVelocity(AngleUnit.DEGREES), EPS);
        assertEquals(2 * Math.PI, motor.getVelocity(AngleUnit.RADIANS), EPS);
        motor.setDirection(Direction.REVERSE);
        motor.setVelocity(-Math.PI, AngleUnit.RADIANS); motor.advance(1);
        assertEquals(-360, motor.getVelocity(), EPS);
        motor.setBatteryVoltage(6); motor.setVelocity(1e6); motor.advance(1);
        assertEquals(600, motor.getVelocity(), EPS);
        assertEquals(RunMode.RUN_USING_ENCODER, motor.getMode());
    }

    @Test public void modeTransitionsReissueCachedPowerAndEncoderPowerCompensatesVoltage() {
        FakeDcMotorEx motor = plant();
        motor.setTimeConstant(0.01);
        motor.setPower(0.2);
        motor.setVelocity(800);
        motor.advance(1);
        assertEquals(800, motor.getVelocity(), EPS);
        motor.setMode(RunMode.RUN_USING_ENCODER);
        motor.advance(1);
        assertEquals(800, motor.getVelocity(), EPS);
        motor.setMode(RunMode.RUN_WITHOUT_ENCODER);
        motor.advance(1);
        assertEquals(200, motor.getVelocity(), EPS);
        motor.setMode(RunMode.RUN_USING_ENCODER);
        motor.setBatteryVoltage(6);
        motor.setPower(0.4);
        motor.advance(1);
        assertEquals(400, motor.getVelocity(), EPS);
        motor.setPower(0.8); motor.advance(1);
        assertEquals(500, motor.getVelocity(), EPS);
        motor.setMode(RunMode.STOP_AND_RESET_ENCODER);
        assertEquals(0.8, motor.getPower(), EPS);
        motor.advance(1);
        assertEquals(0, motor.getExactPosition(), EPS);
        motor.setMode(RunMode.RUN_WITHOUT_ENCODER);
        motor.advance(1);
        assertEquals(400, motor.getVelocity(), EPS);
        motor.setMode(RunMode.STOP_AND_RESET_ENCODER);
        motor.setEncoderPosition(25);
        motor.setMode(RunMode.STOP_AND_RESET_ENCODER);
        assertEquals(25, motor.getCurrentPosition());
    }

    @Test public void pidAndMotorConfigurationsAreDefensiveAndModeSpecific() {
        FakeDcMotorEx motor = plant();
        PIDFCoefficients input = new PIDFCoefficients(1, 2, 3, 4, MotorControlAlgorithm.PIDF);
        motor.setPIDFCoefficients(RunMode.RUN_USING_ENCODER, input);
        input.p = 999;
        PIDFCoefficients output = motor.getPIDFCoefficients(RunMode.RUN_USING_ENCODER);
        assertEquals(1, output.p, EPS); output.p = 123;
        assertEquals(1, motor.getPIDCoefficients(RunMode.RUN_USING_ENCODER).p, EPS);
        motor.setPIDCoefficients(RunMode.RUN_TO_POSITION, new PIDCoefficients(5, 6, 7));
        assertEquals(MotorControlAlgorithm.LegacyPID, motor.getPIDFCoefficients(RunMode.RUN_TO_POSITION).algorithm);
        motor.setPositionPIDFCoefficients(8);
        assertEquals(8, motor.getPIDCoefficients(RunMode.RUN_TO_POSITION).p, EPS);
        assertEquals(0, motor.getPIDFCoefficients(RunMode.RUN_TO_POSITION).f, EPS);
        motor.setVelocityPIDFCoefficients(9, 10, 11, 12);
        assertEquals(12, motor.getPIDFCoefficients(RunMode.RUN_USING_ENCODER).f, EPS);
        MotorConfigurationType type = motor.getMotorType();
        type.setTicksPerRev(100);
        assertEquals(560, motor.getMotorType().getTicksPerRev(), EPS);
        motor.setMotorType(type); type.setTicksPerRev(200);
        assertEquals(100, motor.getMotorType().getTicksPerRev(), EPS);
        assertThrows(UnsupportedOperationException.class, () -> motor.getPIDCoefficients(RunMode.RUN_WITHOUT_ENCODER));
        assertThrows(IllegalArgumentException.class, () -> motor.setVelocityPIDFCoefficients(Double.NaN, 0, 0, 0));
        assertThrows(UnsupportedOperationException.class, () -> motor.setPIDFCoefficients(
                RunMode.RUN_TO_POSITION, new PIDFCoefficients(1, 2, 0, 0)));
        assertThrows(UnsupportedOperationException.class, () -> motor.setPIDFCoefficients(
                RunMode.RUN_USING_ENCODER, new PIDFCoefficients(1, 0, 0, 1, MotorControlAlgorithm.LegacyPID)));
        assertThrows(UnsupportedOperationException.class, () -> motor.setPIDFCoefficients(
                RunMode.RUN_USING_ENCODER, new PIDFCoefficients(1, 0, 0, 0, MotorControlAlgorithm.Unknown)));
    }

    @Test public void loadsStallsFaultsEnableAndCurrentsHaveExplicitEffects() {
        FakeDcMotorEx motor = plant();
        motor.setTimeConstant(0.01);
        motor.setLoad(0.5); motor.setPower(1); motor.advance(1);
        assertEquals(500, motor.getVelocity(), EPS);
        motor.setCurrentModel(0.2, 8);
        assertEquals(4.1, motor.getCurrent(CurrentUnit.AMPS), EPS);
        motor.setStalled(true);
        double position = motor.getExactPosition();
        motor.advance(1);
        assertEquals(position, motor.getExactPosition(), EPS);
        assertEquals(8, motor.getCurrent(CurrentUnit.AMPS), EPS);
        motor.setCurrentAlert(7000, CurrentUnit.MILLIAMPS);
        assertEquals(7, motor.getCurrentAlert(CurrentUnit.AMPS), EPS);
        assertTrue(motor.isOverCurrent());
        motor.setFaulted(true); motor.advance(1);
        assertEquals(0, motor.getCurrent(CurrentUnit.AMPS), EPS);
        motor.setCurrentAmps(2.5);
        assertEquals(2500, motor.getCurrent(CurrentUnit.MILLIAMPS), EPS);
        motor.clearCurrentOverride();
        motor.setFaulted(false); motor.setStalled(false);
        motor.setMotorDisable(); motor.advance(1);
        assertFalse(motor.isMotorEnabled());
        assertEquals(position, motor.getExactPosition(), EPS);
        motor.setMotorEnable(); motor.advance(1);
        assertTrue(motor.getExactPosition() > position);
        motor.setMotorDisable(); motor.setPower(0.5);
        assertTrue(motor.isMotorEnabled());
    }

    @Test public void brakeAndFloatCoastWithExactIntegration() {
        FakeDcMotorEx brake = plant(), floating = plant();
        brake.setMeasuredVelocity(100); floating.setMeasuredVelocity(100);
        brake.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        floating.setPowerFloat();
        brake.advance(0.1); floating.advance(0.1);
        assertEquals(100 * Math.exp(-4), brake.getVelocity(), EPS);
        assertEquals(100 * Math.exp(-0.2), floating.getVelocity(), EPS);
        assertTrue(floating.getPowerFloat());
        assertTrue(brake.getExactPosition() < floating.getExactPosition());
        FakeDcMotorEx split = plant();
        split.setMeasuredVelocity(100);
        for (int i = 0; i < 10; i++) split.advance(0.01);
        assertEquals(floating.getExactPosition(), split.getExactPosition(), EPS);
    }

    @Test public void historiesAreImmutableSnapshotsAndCountWritesSeparately() {
        FakeDcMotorEx motor = new FakeDcMotorEx();
        motor.setPower(0.4);
        List<Double> powerSnapshot = motor.getPowerHistory();
        List<FakeDcMotorEx.Call> calls = motor.getCallHistory();
        List<FakeDcMotorEx.Call> writes = motor.getWriteHistory();
        motor.getPower(); motor.advance(0.1); motor.setPower(2);
        assertEquals(1, calls.size()); assertEquals(1, writes.size());
        assertEquals(1, powerSnapshot.size());
        assertEquals(2, motor.getPowerWriteCount());
        assertEquals(2, motor.getWriteCount("setPower"));
        assertEquals(1, motor.getCallCount("getPower"));
        assertEquals(4, motor.getCallCount());
        assertThrows(UnsupportedOperationException.class, calls::clear);
        assertThrows(UnsupportedOperationException.class, writes::clear);
        assertThrows(UnsupportedOperationException.class, () -> powerSnapshot.add(0.0));
        assertThrows(UnsupportedOperationException.class, () -> calls.get(0).arguments.add(0.0));
    }

    @Test public void realSdkWrapperUsesRawControllerStateWithoutDoubleDirectionOrStep() {
        FakeDcMotorController controller = new FakeDcMotorController(2);
        FakeDcMotorEx plant = controller.getMotor(0);
        MotorConfigurationType type = plant.getMotorType();
        DcMotorEx wrapper;
        try (org.firstinspires.ftc.teamcode.testing.AndroidLogScope logging =
                     new org.firstinspires.ftc.teamcode.testing.AndroidLogScope()) {
            wrapper = new DcMotorImplEx(controller, 0, Direction.REVERSE, type);
        }
        plant.setTimeConstant(0.01);
        wrapper.setPower(0.5);
        plant.advance(1);
        assertEquals(-0.5, controller.getMotorPower(0), EPS);
        assertEquals(0.5, wrapper.getPower(), EPS);
        assertTrue(wrapper.getCurrentPosition() > 0);
        assertTrue(controller.getMotorCurrentPosition(0) < 0);
        wrapper.setVelocity(360, AngleUnit.DEGREES); plant.advance(1);
        assertEquals(560, wrapper.getVelocity(), EPS);
        assertEquals(-560, controller.getMotorVelocity(0), EPS);
        assertEquals(360, wrapper.getVelocity(AngleUnit.DEGREES), EPS);
        wrapper.setMode(RunMode.STOP_AND_RESET_ENCODER);
        wrapper.setTargetPositionTolerance(2);
        wrapper.setTargetPosition(1000);
        wrapper.setMode(RunMode.RUN_TO_POSITION); wrapper.setPower(-1); plant.advance(5);
        assertFalse(wrapper.isBusy());
        assertEquals(998, wrapper.getCurrentPosition());
        assertEquals(-1000, controller.getMotorTargetPosition(0));
        wrapper.setVelocityPIDFCoefficients(1, 2, 3, 4);
        assertEquals(4, controller.getPIDFCoefficients(0, RunMode.RUN_USING_ENCODER).f, EPS);
        wrapper.setCurrentAlert(3000, CurrentUnit.MILLIAMPS);
        plant.setCurrentAmps(4);
        assertTrue(wrapper.isOverCurrent());
        wrapper.setMotorDisable(); assertFalse(wrapper.isMotorEnabled());
        assertSame(plant, controller.getMotor(0));
        assertEquals(0, controller.getMotor(1).getCurrentPosition());
        List<FakeDcMotorController.Call> history = controller.getWriteHistory();
        assertThrows(UnsupportedOperationException.class, history::clear);
        assertThrows(UnsupportedOperationException.class, () -> history.get(0).arguments.clear());
        assertEquals(2, controller.getWriteCount("setMotorPower"));
        assertFalse(((Object) controller) instanceof FakeSteppable);
    }

    @Test public void invalidInputUnknownPortsAndClosedDevicesFailLoudly() {
        FakeDcMotorEx motor = plant();
        assertThrows(IllegalArgumentException.class, () -> motor.advance(-1));
        assertThrows(IllegalArgumentException.class, () -> motor.advance(Double.NaN));
        assertThrows(IllegalArgumentException.class, () -> motor.setPower(Double.NaN));
        assertThrows(IllegalArgumentException.class, () -> motor.setVelocity(Double.POSITIVE_INFINITY));
        assertThrows(IllegalArgumentException.class, () -> motor.setBatteryVoltage(-1));
        assertThrows(IllegalArgumentException.class, () -> motor.setTimeConstant(0));
        assertThrows(IllegalArgumentException.class, () -> motor.setLoad(1.1));
        assertThrows(IllegalArgumentException.class, () -> motor.setCurrentAlert(-1, CurrentUnit.AMPS));
        assertThrows(NullPointerException.class, () -> motor.setMode(null));
        assertThrows(NullPointerException.class, () -> motor.getVelocity(null));
        FakeDcMotorController controller = motor.getController();
        assertThrows(IllegalArgumentException.class, () -> controller.getMotor(1));
        assertThrows(IllegalArgumentException.class, () -> controller.getMotor(-1));
        assertThrows(IllegalStateException.class, () -> new FakeDcMotorEx(controller, 0));
        motor.close();
        assertThrows(IllegalStateException.class, () -> motor.advance(1));
        assertThrows(IllegalStateException.class, () -> controller.setMotorPower(0, 1));
        FakeDcMotorEx other = plant();
        FakeDcMotorController otherController = other.getController();
        otherController.close();
        assertTrue(other.isClosed());
        assertThrows(IllegalStateException.class, () -> other.setPower(1));
        assertThrows(IllegalStateException.class, () -> otherController.getMotor(0));
    }
}
