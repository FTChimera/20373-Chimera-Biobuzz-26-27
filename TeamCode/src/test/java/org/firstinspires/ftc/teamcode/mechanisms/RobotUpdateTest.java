package org.firstinspires.ftc.teamcode.mechanisms;

import static org.junit.Assert.*;
import static org.mockito.ArgumentMatchers.*;
import static org.mockito.Mockito.*;

import com.pedropathing.math.Velocity;
import com.rmsftc.utils.AllianceColor;
import com.rmsftc.utils.DriveVector;
import com.rmsftc.utils.RobotMode;
import com.rmsftc.utils.math.Pose;

import org.firstinspires.ftc.teamcode.fakeHw.FakeDcMotorEx;
import org.firstinspires.ftc.teamcode.fakeHw.FakeHardwareMap;
import org.firstinspires.ftc.teamcode.fakeHw.FakeServo;
import org.firstinspires.ftc.teamcode.fakeHw.FakeVoltageSensor;
import org.firstinspires.ftc.teamcode.pedro.FollowerTestRig;
import org.firstinspires.ftc.teamcode.planners.Constants;
import org.firstinspires.ftc.teamcode.testing.FtcSimulationRule;
import org.firstinspires.ftc.teamcode.testing.RecordingTelemetry;
import org.firstinspires.ftc.teamcode.testing.RobotTestRig;
import org.firstinspires.ftc.teamcode.util.FakeClock;
import org.firstinspires.ftc.teamcode.util.game.Field;
import org.junit.After;
import org.junit.Rule;
import org.junit.Test;
import org.mockito.InOrder;
import org.mockito.stubbing.Answer;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/** Real synchronous Robot loop; spies observe orchestration without replacing mechanism logic. */
public class RobotUpdateTest {
    @Rule public final FtcSimulationRule simulation = new FtcSimulationRule();
    private final List<LoopRig> rigs = new ArrayList<>();

    @After public void stopRobots() {
        for (LoopRig rig : rigs) rig.robot.stopEverything();
    }

    private LoopRig rig() { return new LoopRig(simulation.clock); }

    private final class LoopRig {
        final FakeClock clock;
        final FollowerTestRig drive = new FollowerTestRig();
        final RecordingTelemetry telemetry = new RecordingTelemetry();
        final FakeVoltageSensor battery = spy(new FakeVoltageSensor(12));
        final Robot robot;

        LoopRig(FakeClock clock) {
            this.clock = clock;
            FakeHardwareMap hardware = new FakeHardwareMap();
            for (String name : new String[]{"intake", "transfer", "mainShooter", "counterShooter"}) {
                hardware.register(name, new FakeDcMotorEx());
            }
            hardware.register("deployer", new FakeServo());
            hardware.register("battery", battery);
            Pose goal = Field.HiveState.UNTIPPED.goalPose(AllianceColor.RED);
            robot = spy(new Robot(telemetry, hardware, AllianceColor.RED, RobotMode.TELEOP,
                    new Pose(goal.x() - 40, goal.y(), 0), clock, ignored -> drive.follower));
            robot.follower = spy(robot.follower);
            robot.vision = spy(robot.vision);
            robot.field = spy(robot.field);
            robot.autoAimerCalculator = spy(robot.autoAimerCalculator);
            robot.intake = spy(robot.intake);
            robot.transfer = spy(robot.transfer);
            robot.shooter = spy(robot.shooter);
            robot.lights = spy(robot.lights);
            rigs.add(this);
        }

        Object[] observed() {
            return new Object[]{robot, robot.follower, robot.vision, robot.field,
                    robot.autoAimerCalculator, robot.intake, robot.transfer,
                    robot.shooter, robot.lights, battery};
        }

        void clearCalls() { clearInvocations(observed()); }

        void assertOneTick(boolean smartIntake, boolean smartShooter, boolean debug) {
            verify(robot.follower).update(anyDouble());
            verify(robot.follower, never()).update();
            verify(robot.vision).update(anyDouble());
            verify(robot.field).setHiveState(any(Field.HiveState.class));
            verify(robot.autoAimerCalculator).setHiveState(any(Field.HiveState.class));
            verify(robot.autoAimerCalculator).updatePedro(any(), any(), anyDouble(), anyBoolean());
            verify(robot, times(smartIntake ? 1 : 0)).smartIntakeUpdate();
            verify(robot, times(smartShooter ? 1 : 0)).smartShooterUpdate();
            verify(robot.intake).update();
            verify(robot.transfer).update();
            verify(robot.shooter).update();
            verify(robot.lights).update();
            verify(robot, times(debug ? 1 : 0)).debug();
        }
    }

    @Test public void startAndUpdateRunEveryStageExactlyOnceInOrderOnCallingThread() {
        LoopRig rig = rig();
        Robot robot = rig.robot;
        robot.start();
        robot.update();
        rig.clearCalls();
        simulation.clock.add(20_000_000);
        Pose pose = Field.HiveState.TIPPED.goalPose(AllianceColor.RED).withHeading(0.7);
        rig.drive.localizer.setPose(pose);
        rig.drive.velocity = new Velocity(2, 3, 0);
        Shooter.useSotm.set(true);
        robot.setSmartIntakeControl(true);
        robot.setSmartShooterControl(true);
        robot.debug = true;
        robot.setTeleOpDrive(new DriveVector(0.4, 0.2, 0), false, false);
        Thread caller = Thread.currentThread();
        Answer<Object> synchronous = call -> {
            assertSame(caller, Thread.currentThread());
            return call.callRealMethod();
        };
        doAnswer(synchronous).when(robot.vision).update(anyDouble());
        doAnswer(synchronous).when(robot.field).setHiveState(any());
        doAnswer(synchronous).when(robot.autoAimerCalculator).setHiveState(any());
        doAnswer(synchronous).when(robot.autoAimerCalculator)
                .updatePedro(any(), any(), anyDouble(), anyBoolean());
        doAnswer(synchronous).when(robot).smartIntakeUpdate();
        doAnswer(synchronous).when(robot).smartShooterUpdate();
        doAnswer(synchronous).when(robot.intake).update();
        doAnswer(synchronous).when(robot.transfer).update();
        doAnswer(synchronous).when(robot.shooter).update();
        doAnswer(synchronous).when(robot.lights).off();
        doAnswer(synchronous).when(robot.lights).update();
        doAnswer(synchronous).when(robot).debug();
        doAnswer(call -> {
            assertSame(caller, Thread.currentThread());
            assertEquals("clock advances before battery", 0.02, robot.getDt(), 1e-12);
            return call.callRealMethod();
        }).when(robot).getBatteryVoltage();
        doAnswer(call -> {
            assertSame(caller, Thread.currentThread());
            assertFalse("queued drive is cleared before follower", robot.setTeleOpDrive);
            assertEquals(Field.HiveState.TIPPED, robot.field.getHiveState());
            assertEquals(pose.x() + 2 * 0.02,
                    robot.autoAimerCalculator.getCorrectedPose().x(), 1e-12);
            return call.callRealMethod();
        }).when(robot.follower).update(anyDouble());

        robot.update();

        InOrder order = inOrder(rig.observed());
        order.verify(robot).getBatteryVoltage();
        order.verify(robot.follower).manual(same(robot.drivePowers));
        order.verify(robot.vision).update(pose.heading());
        order.verify(robot.field).setHiveState(Field.HiveState.TIPPED);
        order.verify(robot.autoAimerCalculator).setHiveState(Field.HiveState.TIPPED);
        order.verify(robot.autoAimerCalculator).updatePedro(argThat(actual ->
                        actual.x() == pose.x() && actual.y() == pose.y()
                                && actual.heading() == pose.heading()), same(rig.drive.velocity),
                eq(0.02), eq(true));
        order.verify(robot.follower).update(0.02);
        order.verify(robot).smartIntakeUpdate();
        order.verify(robot).smartShooterUpdate();
        order.verify(robot.intake).update();
        order.verify(robot.transfer).update();
        order.verify(robot.shooter).update();
        order.verify(robot.lights).off();
        order.verify(robot.lights).update();
        order.verify(robot).debug();
        rig.assertOneTick(true, true, true);
        verify(robot, times(2)).getBatteryVoltage();
        robot.telemetry.update();
        assertTrue(rig.telemetry.latest().values().containsKey("=== UTILS DEBUG ==="));
    }

    @Test public void queuedDriveSelectsHoldOrManualAndIsConsumedOnlyOnce() {
        LoopRig rig = rig();
        Robot robot = rig.robot;
        robot.start();
        rig.clearCalls();
        robot.setTeleOpDrive(new DriveVector(0, 0, 0), false, true);
        verify(robot.follower, never()).driveOrHold(any());
        verify(robot.follower, never()).manual(any());
        assertTrue(robot.setTeleOpDrive);
        robot.update();
        verify(robot.follower).driveOrHold(same(robot.drivePowers));
        verify(robot.follower, never()).manual(any());
        assertTrue(robot.follower.holding());
        assertFalse(robot.setTeleOpDrive);
        rig.clearCalls();
        robot.update();
        verify(robot.follower, never()).driveOrHold(any());
        verify(robot.follower, never()).manual(any());

        rig.clearCalls();
        robot.setTeleOpDrive(new DriveVector(0.4, -0.3, 0.2), false, false);
        verify(robot.follower, never()).manual(any());
        robot.update();
        verify(robot.follower).manual(same(robot.drivePowers));
        verify(robot.follower, never()).driveOrHold(any());
        assertTrue(robot.follower.manual());
        assertSame(robot.drivePowers, robot.follower.manualPowers);
        assertFalse(robot.setTeleOpDrive);
        rig.clearCalls();
        robot.update();
        verify(robot.follower, never()).manual(any());
        verify(robot.follower, never()).driveOrHold(any());
    }

    @Test public void fakeClockSuppliesFirstZeroDtLaterDeltasAndRestartedOpModeTime() {
        simulation.clock.set(5_000_000_000L);
        LoopRig rig = rig();
        Robot robot = rig.robot;
        robot.start();
        simulation.clock.add(100_000_000);
        robot.update();
        assertEquals(0, robot.getDt(), 0);
        assertEquals(0.1, robot.getOpModeTimeSeconds(), 1e-12);
        assertEquals(robot.robotMode.getTimeLimit() - 0.1, robot.opModeTimeRemaining(), 1e-12);
        verify(robot.follower).update(0);
        for (long delta : new long[]{20_000_000, 37_000_000}) {
            rig.clearCalls();
            simulation.clock.add(delta);
            robot.update();
            assertEquals(delta / 1e9, robot.getDt(), 1e-12);
            verify(robot.follower).update(delta / 1e9);
            verify(robot.autoAimerCalculator).updatePedro(any(), any(), eq(delta / 1e9),
                    eq(Shooter.useSotm.get()));
        }
        assertEquals(0.157, robot.getOpModeTimeSeconds(), 1e-12);
        robot.stopEverything();
        simulation.clock.add(1_000_000_000);
        robot.start();
        assertEquals(0, robot.getOpModeTimeSeconds(), 0);
        rig.clearCalls();
        robot.update();
        assertEquals(0, robot.getDt(), 0);
        verify(robot.follower).update(0);
    }

    @Test public void headingOverrideExistsOnlyForRequestedTickAndReleasesBeforeFollower() {
        LoopRig rig = rig();
        Robot robot = rig.robot;
        robot.start();
        robot.update();
        robot.follower.follow(FollowerTestRig.path());
        List<Double> headingsAtFollower = new ArrayList<>();
        doAnswer(call -> {
            headingsAtFollower.add(robot.follower.headingOverride);
            return call.callRealMethod();
        }).when(robot.follower).update(anyDouble());
        robot.setAutoAlign();
        Double target = robot.follower.headingOverride;
        assertNotNull(target);
        assertFalse(robot.setTeleOpDrive);
        robot.update();
        assertEquals(target, robot.follower.headingOverride);
        assertEquals(target, rig.drive.pathHeading, 1e-12);
        robot.update();
        assertNull(robot.follower.headingOverride);
        assertEquals(0, rig.drive.pathHeading, 0);
        robot.setAutoAlign();
        robot.update();
        assertNotNull(robot.follower.headingOverride);
        robot.update();
        assertEquals(Arrays.asList(target, null, target, null), headingsAtFollower);
        assertTrue(robot.follower.following());
    }

    @Test public void smartControlsGateIndependentlyAndDebugIsOptional() {
        LoopRig rig = rig();
        Robot robot = rig.robot;
        robot.start();
        for (int flags = 0; flags < 4; flags++) {
            boolean intake = (flags & 1) != 0;
            boolean shooter = (flags & 2) != 0;
            robot.setSmartIntakeControl(intake);
            robot.setSmartShooterControl(shooter);
            robot.debug = flags == 3;
            rig.clearCalls();
            robot.update();
            rig.assertOneTick(intake, shooter, robot.debug);
            verify(robot.shooter, times(shooter ? 1 : 0))
                    .solveAndSetVelocity(any(), any(), any(), anyDouble());
            if (!robot.debug) {
                robot.telemetry.update();
                assertTrue(rig.telemetry.latest().entries.isEmpty());
            }
        }
        robot.telemetry.update();
        assertTrue(rig.telemetry.latest().values().containsKey("=== FOLLOWER DEBUG ==="));
    }

    @Test public void cachedBatteryRefreshPrecedesSmartIntakeAndPreservesEject() {
        LoopRig rig = rig();
        Robot robot = rig.robot;
        robot.start();
        robot.setSmartIntakeControl(true);
        robot.update();
        assertEquals(Intake.Mode.FULL_POWER, robot.intake.mode());
        double low = Constants.Robot.Battery.LOW_POWER_VOLTAGE_THRESHOLD
                - Constants.Robot.Battery.LOW_POWER_VOLTAGE_TOLERANCE - 0.1;
        rig.battery.setVoltage(low);
        simulation.clock.add(1_000_000);
        robot.update();
        verify(rig.battery).getVoltage();
        assertEquals(12, robot.lastVolt, 0);
        assertEquals(Intake.Mode.FULL_POWER, robot.intake.mode());
        simulation.clock.add(Constants.Robot.Battery.CACHE_MS * 1_000_000L);
        robot.update();
        verify(rig.battery, times(2)).getVoltage();
        assertEquals(low, robot.lastVolt, 0);
        assertEquals(Intake.Mode.LOW_POWER, robot.intake.mode());
        robot.intake.forceEject();
        robot.update();
        assertEquals(Intake.Mode.EJECTING, robot.intake.mode());
    }

    @Test public void lightsControlChoosesStatusBeforeUpdatingLights() {
        LoopRig rig = rig();
        Robot robot = rig.robot;
        robot.start();
        rig.clearCalls();
        robot.update();
        InOrder order = inOrder(robot.lights);
        order.verify(robot.lights).off();
        order.verify(robot.lights).update();
        doReturn(true).when(robot.shooter).upToSpeed();
        rig.clearCalls();
        robot.update();
        order = inOrder(robot.lights);
        order.verify(robot.lights).displayGreen();
        order.verify(robot.lights).update();
        simulation.clock.set((long) ((robot.robotMode.getTimeLimit()
                - Constants.Robot.Lights.ONE_MINUTE_WINDOW_START_SEC) * 1e9));
        rig.clearCalls();
        robot.update();
        order = inOrder(robot.lights);
        order.verify(robot.lights).oneMinThing();
        order.verify(robot.lights).update();
        simulation.clock.set((long) ((robot.robotMode.getTimeLimit()
                - Constants.Robot.Lights.FLASH_RED_SECONDS_REMAINING) * 1e9));
        rig.clearCalls();
        robot.update();
        order = inOrder(robot.lights);
        order.verify(robot.lights).flashRed();
        order.verify(robot.lights).update();
    }

    @Test public void lifecycleGuardAndStopKeepAllRealMotorOutputsZero() {
        try (RobotTestRig rig = new RobotTestRig(simulation.clock)) {
            Robot robot = rig.createRobot(new RecordingTelemetry(), AllianceColor.RED,
                    RobotMode.TELEOP, new Pose(50, 50, 0));
            assertThrows(IllegalStateException.class, robot::update);
            for (FakeDcMotorEx motor : rig.motors) assertEquals(0, motor.getPower(), 0);
            robot.start();
            robot.setTeleOpDrive(new DriveVector(0.5, 0, 0), false, false);
            robot.intake.setFullPower();
            robot.transfer.reverse();
            robot.shooter.reverseOuttake();
            robot.update();
            assertTrue(rig.driveMotors.stream().anyMatch(m -> m.getPower() != 0));
            for (FakeDcMotorEx motor : new FakeDcMotorEx[]{rig.intake, rig.transfer,
                    rig.mainShooter, rig.counterShooter}) {
                assertNotEquals("shutdown stimulus must energize each mechanism", 0, motor.getPower(), 0);
            }
            robot.setSmartIntakeControl(true);
            robot.setSmartShooterControl(true);
            robot.setTeleOpDrive(new DriveVector(0.5, 0, 0), false, false);
            robot.stopEverything();
            assertFalse(robot.smartIntakeControl);
            assertFalse(robot.smartShooterControl);
            assertFalse(robot.setTeleOpDrive);
            for (int tick = 0; tick < 3; tick++) {
                simulation.clock.add(20_000_000);
                rig.advance(0.02);
                assertThrows(IllegalStateException.class, robot::update);
                for (FakeDcMotorEx motor : rig.motors) assertEquals(0, motor.getPower(), 0);
            }
        }
    }

    @Test public void twoRobotsUpdateTheirOwnSubsystemsAndClocksIndependently() {
        LoopRig first = new LoopRig(new FakeClock());
        LoopRig second = new LoopRig(new FakeClock());
        first.robot.start();
        second.robot.start();
        first.clearCalls();
        second.clearCalls();
        first.robot.update();
        first.assertOneTick(false, false, false);
        verifyNoInteractions(second.robot.follower, second.robot.vision,
                second.robot.autoAimerCalculator, second.robot.intake,
                second.robot.transfer, second.robot.shooter, second.robot.lights);
        first.clearCalls();
        second.robot.update();
        second.assertOneTick(false, false, false);
        verifyNoInteractions(first.robot.follower, first.robot.vision, first.robot.intake);
        first.clearCalls();
        second.clearCalls();
        first.clock.add(20_000_000);
        second.clock.add(37_000_000);
        first.robot.update();
        second.robot.update();
        verify(first.robot.follower).update(0.02);
        verify(second.robot.follower).update(0.037);
        first.robot.stopEverything();
        second.clearCalls();
        second.clock.add(11_000_000);
        second.robot.update();
        second.assertOneTick(false, false, false);
        assertEquals(0.011, second.robot.getDt(), 1e-12);
    }
}
