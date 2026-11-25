package org.firstinspires.ftc.teamcode.mechanisms;

import static org.junit.Assert.*;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.rmsftc.utils.AllianceColor;
import org.firstinspires.ftc.teamcode.fakeHw.FakeDcMotorEx;
import org.firstinspires.ftc.teamcode.fakeHw.FakeHardwareMap;
import org.firstinspires.ftc.teamcode.fakeHw.FakeVoltageSensor;
import org.firstinspires.ftc.teamcode.planners.Constants;
import org.firstinspires.ftc.teamcode.testing.FtcSimulationRule;
import org.firstinspires.ftc.teamcode.util.FakeClock;
import org.firstinspires.ftc.teamcode.util.Timer;
import org.firstinspires.ftc.teamcode.util.control.Controller;
import org.firstinspires.ftc.teamcode.util.control.DoubleFlywheelFullVelocityConstants;
import org.firstinspires.ftc.teamcode.util.control.DoubleFlywheelFullVelocityController;
import org.firstinspires.ftc.teamcode.util.hardware.actuator.SmartFlywheel;
import org.junit.Rule;
import org.junit.Test;

import java.util.Collections;
import java.util.concurrent.TimeUnit;

/** Deterministic synthetic-plant tests, not evidence that competition gains are stable. */
public class FlywheelSimulationTest {
    @Rule public final FtcSimulationRule simulation = new FtcSimulationRule();

    /**
     * Synthetic shaft: dv/dt = (2000 * power * battery/12 * (1-load) - v)/0.12.
     * The fake integrates that first-order response exactly at 20 ms steps.
     * kV=12/2000 volts/(ticks/s), kP=.0004 power/(ticks/s),
     * kI=.0005 power/tick; no static/derivative/acceleration term.
     * These deliberately benign gains belong ONLY to this model, not the robot.
     */
    private static DoubleFlywheelFullVelocityConstants plantGains() {
        return new DoubleFlywheelFullVelocityConstants(
                0.0004, 0.0005, 0, 0, 0.006, 0, 25, 1000, true, 12);
    }

    static FakeDcMotorEx plantMotor() {
        FakeDcMotorEx motor = new FakeDcMotorEx();
        motor.setFreeSpeedTicksPerSecond(2000);
        motor.setTimeConstant(0.12);
        motor.setNominalVoltage(12);
        return motor;
    }

    static FakeHardwareMap hardwareMap() {
        return new FakeHardwareMap();
    }

    private final class ShooterRig {
        final FakeDcMotorEx main = plantMotor();
        final FakeDcMotorEx counter = plantMotor();
        final FakeVoltageSensor battery = new FakeVoltageSensor(12);
        final FakeHardwareMap hardware = hardwareMap();
        final Shooter shooter;

        ShooterRig() {
            hardware.register("mainShooter", main);
            hardware.register("counterShooter", counter);
            hardware.register("battery", battery);
            shooter = new Shooter(hardware, AllianceColor.RED, simulation.clock);
            configureSyntheticGains(shooter.mainMotor);
            configureSyntheticGains(shooter.counterRoller);
        }

        void voltage(double volts) {
            battery.setVoltage(volts);
            main.setBatteryVoltage(volts);
            counter.setBatteryVoltage(volts);
        }

        void step(int count) {
            for (int i = 0; i < count; i++) {
                main.advance(0.02);
                counter.advance(0.02);
                simulation.clock.add(20_000_000);
                shooter.update();
                assertBounded(main);
                assertBounded(counter);
                assertControllerBounded(shooter.mainMotor);
                assertControllerBounded(shooter.counterRoller);
            }
        }

        void assertReady() {
            assertEquals(1200, main.getVelocity(), 25);
            assertEquals(900, counter.getVelocity(), 25);
            assertTrue("both real wheel measurements must reach their targets", shooter.upToSpeed());
        }
    }

    static void configureSyntheticGains(SmartFlywheel wheel) {
        DoubleFlywheelFullVelocityConstants c =
                ((DoubleFlywheelFullVelocityController) wheel.velocityController).getConstants();
        DoubleFlywheelFullVelocityConstants synthetic = plantGains();
        c.kP = synthetic.kP;
        c.kI = synthetic.kI;
        c.kD = synthetic.kD;
        c.kS = synthetic.kS;
        c.kV = synthetic.kV;
        c.kA = synthetic.kA;
        c.tolerance = synthetic.tolerance;
        c.integralLimit = synthetic.integralLimit;
        c.nominalVoltage = synthetic.nominalVoltage;
        c.voltageCompensation = synthetic.voltageCompensation;
        wheel.velocityController.reset();
    }

    private static void assertBounded(FakeDcMotorEx motor) {
        assertTrue("motor power must be finite", Double.isFinite(motor.getPower()));
        assertTrue("motor power must remain in [-1,1]", Math.abs(motor.getPower()) <= 1);
    }

    static void assertControllerBounded(SmartFlywheel wheel) {
        double output = wheel.velocityController.getOutput();
        assertTrue("controller output must be finite before hardware clamping", Double.isFinite(output));
        assertTrue("controller itself must bound power, not rely on CachedMotor",
                Math.abs(output) <= 1);
    }

    @Test public void smartFlywheelConvergesThroughRealClockControllerAndEncoderPlant() {
        FakeDcMotorEx motor = plantMotor();
        DoubleFlywheelFullVelocityController controller =
                new DoubleFlywheelFullVelocityController(plantGains(), simulation.clock);
        SmartFlywheel wheel = new SmartFlywheel(motor, controller,
                new SmartFlywheel.Params(28, 1),
                Collections.singletonList(new FakeVoltageSensor(12)));
        wheel.setVelocity(1200);
        wheel.update();
        assertEquals(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER, motor.getMode());
        assertEquals(DcMotorEx.ZeroPowerBehavior.FLOAT, motor.getZeroPowerBehavior());
        assertEquals(1, motor.getPower(), 0);
        motor.advance(0.02);
        assertEquals("first-order plant must actually respond to controller output",
                2000 * (1 - Math.exp(-0.02 / 0.12)), motor.getVelocity(), 1e-9);
        for (int i = 0; i < 400; i++) {
            simulation.clock.add(20_000_000);
            wheel.update();
            assertBounded(motor);
            assertControllerBounded(wheel);
            motor.advance(0.02);
        }
        assertEquals(1200, wheel.getTicksPerSecond(), 25);
        assertEquals(1200 * 60.0 / 28, wheel.getRPM(), 25 * 60.0 / 28);
        assertTrue(controller.atTarget());
        assertTrue("exercise changing physical power writes", motor.getPowerWriteCount() > 10);
        assertEquals("velocity feedback is software controlled, never a hub setVelocity mock",
                0, motor.getVelocityWriteCount());
    }

    @Test public void shooterRecoversFromBatterySagWithBothTargetsAndBoundedPower() {
        ShooterRig rig = new ShooterRig();
        rig.shooter.setVelocity(1200, 900);
        assertFalse(rig.shooter.upToSpeed());
        rig.step(400);
        rig.assertReady();
        double before = rig.main.getPower();
        rig.voltage(9.6);
        rig.shooter.update();
        assertTrue("fresh sag reading must increase real feedforward power",
                rig.main.getPower() > before + 0.1);
        rig.step(400);
        rig.assertReady();
        assertEquals(1 / 9.6, ((DoubleFlywheelFullVelocityController)
                rig.shooter.mainMotor.velocityController).getAppliedVoltageScale(), 1e-12);
        rig.voltage(12);
        rig.step(400);
        rig.assertReady();
    }

    @Test public void shooterRejectsStalledWheelThenRecoversAfterLoadAndStallRemoval() {
        ShooterRig rig = new ShooterRig();
        rig.shooter.setVelocity(1200, 900);
        rig.step(400);
        rig.assertReady();
        rig.counter.setLoad(0.3);
        rig.counter.advance(0.12);
        assertTrue("load stimulus must reduce the physical counter-wheel speed",
                rig.counter.getVelocity() < 900 - 40);
        assertFalse(rig.shooter.upToSpeed());
        rig.step(600);
        rig.assertReady();
        rig.main.setStalled(true);
        rig.step(200);
        assertEquals(0, rig.main.getVelocity(), 0);
        assertEquals("real controller saturates against an impossible stalled target",
                1, rig.main.getPower(), 0);
        assertFalse(rig.shooter.upToSpeed());
        rig.main.setStalled(false);
        rig.counter.setLoad(0);
        rig.step(800);
        rig.assertReady();
    }

    @Test public void readinessRequiresBothWheelsAndRejectsEitherExactlyOneZeroTarget() {
        ShooterRig rig = new ShooterRig();
        rig.shooter.setVelocity(1200, 900);
        rig.main.setMeasuredVelocity(1200);
        rig.counter.setMeasuredVelocity(0);
        assertFalse("main alone at speed is insufficient", rig.shooter.upToSpeed());
        rig.counter.setMeasuredVelocity(900);
        rig.main.setMeasuredVelocity(0);
        assertFalse("counter alone at speed is insufficient", rig.shooter.upToSpeed());
        rig.main.setMeasuredVelocity(1200);
        assertTrue(rig.shooter.upToSpeed());
        double tolerance = Constants.Shooter.SmartFlywheel.UP_TO_SPEED_TOLERANCE_TICKS_PER_SEC;
        rig.main.setMeasuredVelocity(1200 + tolerance);
        rig.counter.setMeasuredVelocity(900 - tolerance);
        assertTrue("both inclusive tolerance edges are ready", rig.shooter.upToSpeed());
        rig.main.setMeasuredVelocity(1200 + tolerance + 1);
        assertFalse("main just outside tolerance blocks readiness", rig.shooter.upToSpeed());
        rig.main.setMeasuredVelocity(1200);
        rig.counter.setMeasuredVelocity(900 - tolerance - 1);
        assertFalse("counter just outside tolerance blocks readiness", rig.shooter.upToSpeed());
        for (int[] target : new int[][]{{0, 900}, {1200, 0}, {0, 0}}) {
            rig.shooter.setVelocity(target[0], target[1]);
            rig.main.setMeasuredVelocity(target[0]);
            rig.counter.setMeasuredVelocity(target[1]);
            assertFalse("zero wheel is not a feedable shot", rig.shooter.hasNonzeroVelocityTarget());
            assertFalse("even exact measurements cannot make a zero target ready",
                    rig.shooter.upToSpeed());
        }
    }

    @Test public void stopImmediatelyZerosSameTickAndCannotResumePanicReverseOrVelocity() {
        ShooterRig rig = new ShooterRig();
        for (int mode = 0; mode < 3; mode++) {
            rig.shooter.setVelocity(1200, 900);
            rig.step(40);
            if (mode == 1) rig.shooter.panic();
            if (mode == 2) rig.shooter.reverseOuttake();
            assertTrue("precondition: powered before stopping", Math.abs(rig.main.getPower()) > 0);
            int writes = rig.main.getPowerWriteCount();
            rig.shooter.stopImmediately();
            assertEquals("no update or elapsed time is needed to cut main power", 0, rig.main.getPower(), 0);
            assertEquals("no update or elapsed time is needed to cut counter power", 0, rig.counter.getPower(), 0);
            assertTrue(rig.main.getPowerWriteCount() > writes);
            assertFalse(rig.shooter.panicking());
            assertFalse(rig.shooter.reversing());
            assertFalse(rig.shooter.hasNonzeroVelocityTarget());
            assertFalse(rig.shooter.upToSpeed());
            assertEquals(0, rig.shooter.mainMotor.targetTicksPerSecond, 0);
            assertEquals(0, rig.shooter.counterRoller.targetTicksPerSecond, 0);
            rig.shooter.update();
            rig.step(100);
            assertEquals(0, rig.main.getPower(), 0);
            assertEquals(0, rig.counter.getPower(), 0);
        }
    }

    @Test public void timerAndTransferKickUseStrictFakeClockBoundaryAndRestorePreviousMode() {
        FakeHardwareMap hardware = hardwareMap();
        FakeDcMotorEx motor = plantMotor();
        hardware.register("transfer", motor);
        Transfer transfer = new Transfer(hardware, simulation.clock);
        transfer.slow();
        transfer.update();
        assertEquals(Constants.Transfer.SLOW_POWER, motor.getPower(), 0);
        transfer.kickOne();
        transfer.update();
        Timer timer = new Timer(TimeUnit.MILLISECONDS, simulation.clock);
        long boundary = Math.round(Constants.Transfer.POLLEN_KICK_TIME_MS * 1e6);
        simulation.clock.add(boundary - 1);
        assertFalse(timer.past(Constants.Transfer.POLLEN_KICK_TIME_MS));
        assertEquals((boundary - 1) / 1e9, timer.dt(TimeUnit.SECONDS), 1e-12);
        transfer.update();
        assertEquals(Transfer.Mode.KICK_ONE, transfer.mode());
        assertEquals(Constants.Transfer.KICK_ONE_POWER, motor.getPower(), 0);
        simulation.clock.add(1);
        assertFalse("past is strictly greater, not greater-or-equal",
                timer.past(Constants.Transfer.POLLEN_KICK_TIME_MS));
        transfer.update();
        assertEquals(Transfer.Mode.KICK_ONE, transfer.mode());
        simulation.clock.add(1);
        assertTrue(timer.past(Constants.Transfer.POLLEN_KICK_TIME_MS));
        transfer.update();
        assertEquals(Transfer.Mode.SLOW, transfer.mode());
        assertTrue("restored mode is applied on the following mechanism tick", transfer.isModeSwitch());
        transfer.update();
        assertEquals(Constants.Transfer.SLOW_POWER, motor.getPower(), 0);
        timer.reset();
        assertEquals(0, timer.get(), 0);
        assertEquals(0, timer.dt(), 0);
        transfer.reverse();
        transfer.update();
        transfer.stopImmediately();
        assertEquals(0, motor.getPower(), 0);
        transfer.update();
        assertEquals(Transfer.Mode.IDLE, transfer.mode());
        assertEquals(0, motor.getPower(), 0);
    }

    private static DoubleFlywheelFullVelocityConstants timingGains() {
        return new DoubleFlywheelFullVelocityConstants(
                0.001, 0.002, 0.00001, 0.1, 0.001, 0.00001,
                1, 100, false, 12);
    }

    @Test public void clockUpdateMatchesExplicitDtIncludingRepeatedTimeAndTargetSteps() {
        DoubleFlywheelFullVelocityController timed =
                new DoubleFlywheelFullVelocityController(timingGains(), simulation.clock);
        Controller<Double, Double, Double> factory =
                Controller.doubleFlywheelFullVelocity(timingGains(), simulation.clock);
        DoubleFlywheelFullVelocityController explicit =
                new DoubleFlywheelFullVelocityController(timingGains(), new FakeClock());
        double[] targets = {100, 100, 130, 130, 90, 90};
        double[] states = {0, 20, 45, 45, 80, 85};
        long[] elapsedNanos = {0, 20_000_000, 35_000_000, 0, 15_000_000, 50_000_000};
        for (int i = 0; i < targets.length; i++) {
            simulation.clock.add(elapsedNanos[i]);
            timed.setTarget(targets[i]);
            factory.setTarget(targets[i]);
            explicit.setTarget(targets[i]);
            timed.update(states[i]);
            factory.update(states[i]);
            explicit.update(states[i], elapsedNanos[i] / 1e9);
            assertEquals("timed output at sample " + i,
                    explicit.getOutput(), timed.getOutput(), 1e-12);
            assertEquals("factory must retain injected clock at sample " + i,
                    explicit.getOutput(), factory.getOutput(), 1e-12);
            assertEquals(explicit.lastError(), timed.lastError(), 1e-12);
            assertEquals(explicit.atTarget(), timed.atTarget());
        }
        assertEquals(5, timed.lastError(), 0);
        assertTrue("exercise a nonzero controller output", timed.getOutput() > 0.1);
    }

    @Test public void resetRestoresFirstUpdateSentinelAtZeroAndAfterLongClockGap() {
        for (long resetTime : new long[]{0, 9_000_000_000L}) {
            simulation.clock.set(0);
            DoubleFlywheelFullVelocityController controller =
                    new DoubleFlywheelFullVelocityController(timingGains(), simulation.clock);
            controller.setTarget(100.0);
            assertFalse(controller.atTarget());
            controller.update(0.0);
            assertEquals(0.3, controller.getOutput(), 1e-12);
            simulation.clock.add(20_000_000);
            controller.update(0.0);
            assertEquals("time zero is a valid baseline, not the uninitialized sentinel",
                    0.304, controller.getOutput(), 1e-12);
            simulation.clock.set(resetTime);
            controller.reset();
            assertEquals(100, controller.getTarget(), 0);
            assertEquals(0, controller.getOutput(), 0);
            assertFalse(controller.atTarget());
            controller.update(20.0);
            assertEquals("first update after reset must ignore elapsed pre-reset time",
                    0.28, controller.getOutput(), 1e-12);
            simulation.clock.add(20_000_000);
            controller.update(20.0);
            assertEquals(0.2832, controller.getOutput(), 1e-12);
        }
    }
}
