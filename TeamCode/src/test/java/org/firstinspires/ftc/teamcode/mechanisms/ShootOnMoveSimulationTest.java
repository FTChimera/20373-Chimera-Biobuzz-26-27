package org.firstinspires.ftc.teamcode.mechanisms;

import static org.junit.Assert.*;
import static org.firstinspires.ftc.teamcode.planners.commands.ShooterCommands.ShotOutcome.*;

import com.pedropathing.math.Velocity;
import com.rmsftc.cmdlib.EndCondition;
import com.rmsftc.utils.AllianceColor;
import com.rmsftc.utils.RobotMode;
import com.rmsftc.utils.math.Pose;
import org.firstinspires.ftc.teamcode.fakeHw.FakeDcMotorEx;
import org.firstinspires.ftc.teamcode.fakeHw.FakeHardwareMap;
import org.firstinspires.ftc.teamcode.fakeHw.FakeServo;
import org.firstinspires.ftc.teamcode.fakeHw.FakeVoltageSensor;
import org.firstinspires.ftc.teamcode.pedro.FollowerTestRig;
import org.firstinspires.ftc.teamcode.planners.Constants;
import org.firstinspires.ftc.teamcode.planners.commands.ShooterCommands;
import org.firstinspires.ftc.teamcode.testing.FtcSimulationRule;
import org.firstinspires.ftc.teamcode.testing.RecordingTelemetry;
import org.firstinspires.ftc.teamcode.util.game.Field;
import org.junit.After;
import org.junit.Rule;
import org.junit.Test;

/** Real Robot update loop, table solver, auto-aimer, wheel controllers and shot lifecycle. */
public class ShootOnMoveSimulationTest {
    @Rule public final FtcSimulationRule simulation = new FtcSimulationRule();
    private RobotRig rig;
    @After public void stopRobot() {
        if (rig != null) rig.robot.stopEverything();
    }

    private final class RobotRig {
        final FakeHardwareMap hardware = FlywheelSimulationTest.hardwareMap();
        final FakeDcMotorEx main = FlywheelSimulationTest.plantMotor();
        final FakeDcMotorEx counter = FlywheelSimulationTest.plantMotor();
        final FakeDcMotorEx intake = new FakeDcMotorEx();
        final FakeDcMotorEx transfer = new FakeDcMotorEx();
        final FollowerTestRig localization = new FollowerTestRig();
        final Robot robot;

        RobotRig() {
            hardware.register("mainShooter", main);
            hardware.register("counterShooter", counter);
            hardware.register("intake", intake);
            hardware.register("transfer", transfer);
            hardware.register("deployer", new FakeServo());
            hardware.register("battery", new FakeVoltageSensor(12));
            Pose goal = Field.HiveState.UNTIPPED.goalPose(AllianceColor.RED);
            Pose pose = new Pose(goal.x() - 40, goal.y(), 0);
            robot = new Robot(new RecordingTelemetry(), hardware, AllianceColor.RED,
                    RobotMode.AUTONOMOUS, pose, simulation.clock, ignored -> localization.follower);
            ShootOnMoveSimulationTest.this.rig = this;
            // Keep all mechanism/controller classes real; use only the documented synthetic
            // first-order plant gains from FlywheelSimulationTest, never competition tuning.
            FlywheelSimulationTest.configureSyntheticGains(robot.shooter.mainMotor);
            FlywheelSimulationTest.configureSyntheticGains(robot.shooter.counterRoller);
            robot.setSmartShooterControl(true);
            robot.start();
            robot.update(); // Clock.Helper's first update establishes the time-zero baseline.
        }

        void step(int count) {
            for (int i = 0; i < count; i++) {
                main.advance(0.02);
                counter.advance(0.02);
                intake.advance(0.02);
                transfer.advance(0.02);
                simulation.clock.add(20_000_000);
                robot.update();
                assertTrue(Double.isFinite(main.getPower()) && Math.abs(main.getPower()) <= 1);
                assertTrue(Double.isFinite(counter.getPower()) && Math.abs(counter.getPower()) <= 1);
                FlywheelSimulationTest.assertControllerBounded(robot.shooter.mainMotor);
                FlywheelSimulationTest.assertControllerBounded(robot.shooter.counterRoller);
            }
        }

        void assertNoFeed() {
            assertEquals(Intake.Mode.IDLE, robot.intake.mode());
            assertEquals(Transfer.Mode.IDLE, robot.transfer.mode());
            assertEquals(0, intake.getPower(), 0);
            assertEquals(0, transfer.getPower(), 0);
            for (double power : intake.getPowerHistory()) {
                assertEquals("intake must never have been energized", 0, power, 0);
            }
            for (double power : transfer.getPowerHistory()) {
                assertEquals("transfer must never have been energized", 0, power, 0);
            }
        }
    }

    @Test public void masterSotmFlagMovesHeadingAndBothRealWheelTargetsAndOffRestoresAllThree() {
        Shooter.useSotm.set(false);
        rig = new RobotRig();
        rig.localization.velocity = new Velocity(15, 30, 0);
        rig.step(1);
        assertEquals(Field.HiveState.UNTIPPED, rig.robot.field.getHiveState());
        assertEquals(0.02, rig.robot.getDt(), 1e-12);
        double stationaryHeading = rig.robot.autoAimerCalculator.getTargetHeading();
        double stationaryMain = rig.robot.shooter.mainMotor.targetTicksPerSecond;
        double stationaryCounter = rig.robot.shooter.counterRoller.targetTicksPerSecond;
        assertEquals("flag off aims at the raw bearing despite robot motion", 0, stationaryHeading, 1e-12);
        assertTrue(stationaryMain > 0);
        assertTrue(stationaryCounter > 0);

        Shooter.useSotm.set(true);
        rig.step(1);
        double movingHeading = rig.robot.autoAimerCalculator.getTargetHeading();
        assertTrue("aim must lead against +y drift, not with it",
                movingHeading > Math.PI && movingHeading < 2 * Math.PI - 0.01);
        assertTrue("toward-hive velocity must lower MAIN wheel target",
                rig.robot.shooter.mainMotor.targetTicksPerSecond < stationaryMain - 1);
        assertTrue("toward-hive velocity must also lower COUNTER wheel target",
                rig.robot.shooter.counterRoller.targetTicksPerSecond < stationaryCounter - 1);
        assertTrue(rig.robot.shooter.mainMotor.targetTicksPerSecond > 0);
        assertTrue(rig.robot.shooter.counterRoller.targetTicksPerSecond > 0);
        assertEquals("release prediction belongs to the same master switch",
                rig.robot.getPose().x() + 15 * 0.02,
                rig.robot.autoAimerCalculator.getCorrectedPose().x(), 1e-12);
        assertEquals(rig.robot.getPose().y() + 30 * 0.02,
                rig.robot.autoAimerCalculator.getCorrectedPose().y(), 1e-12);

        Shooter.useSotm.set(false);
        rig.step(1);
        assertEquals(stationaryHeading, rig.robot.autoAimerCalculator.getTargetHeading(), 1e-12);
        assertEquals(stationaryMain, rig.robot.shooter.mainMotor.targetTicksPerSecond, 0);
        assertEquals(stationaryCounter, rig.robot.shooter.counterRoller.targetTicksPerSecond, 0);
        assertEquals(rig.robot.getPose().x(), rig.robot.autoAimerCalculator.getCorrectedPose().x(), 1e-12);
        assertEquals(rig.robot.getPose().y(), rig.robot.autoAimerCalculator.getCorrectedPose().y(), 1e-12);
    }

    @Test public void infeasibleTowardHiveShotSetsBothTargetsZeroAndCommandSkipsWithoutAnyFeed() {
        Shooter.useSotm.set(true);
        rig = new RobotRig();
        // 1000 in/s is an intentionally infeasible stimulus, not a realistic drive speed.
        // At a 20 ms release horizon the robot remains in front of the hive.
        rig.localization.velocity = new Velocity(1000, 0, 0);
        rig.step(1);
        assertEquals(0, rig.robot.shooter.mainMotor.targetTicksPerSecond, 0);
        assertEquals(0, rig.robot.shooter.counterRoller.targetTicksPerSecond, 0);
        assertFalse(rig.robot.shooter.hasNonzeroVelocityTarget());
        assertFalse(rig.robot.shooter.upToSpeed());
        ShooterCommands.ShotCommand command = ShooterCommands.shootAllPollen(rig.robot);
        command.start();
        command.loop();
        assertEquals(WAITING_FOR_SPEED, command.outcome());
        assertTrue(command.status().isRunning());
        rig.assertNoFeed();

        long timeoutNanos = Constants.Timings.upToSpeedShooterTimeoutTimeMs * 1_000_000L;
        long elapsed = 0;
        while (elapsed + 20_000_000 <= timeoutNanos) {
            rig.step(1);
            elapsed += 20_000_000;
            command.loop();
            assertEquals(WAITING_FOR_SPEED, command.outcome());
            rig.assertNoFeed();
        }
        simulation.clock.add(timeoutNanos - elapsed);
        command.loop();
        assertEquals("strict timer boundary must not feed or skip early",
                WAITING_FOR_SPEED, command.outcome());
        rig.assertNoFeed();
        simulation.clock.add(1);
        command.loop();
        assertEquals(SKIPPED, command.outcome());
        assertFalse(command.status().isRunning());
        assertFalse(command.attempted());
        command.end(EndCondition.NATURALLY);
        rig.step(100);
        command.loop();
        assertEquals("terminal skip persists after later robot ticks", SKIPPED, command.outcome());
        assertEquals(0, rig.robot.shooter.mainMotor.targetTicksPerSecond, 0);
        assertEquals(0, rig.robot.shooter.counterRoller.targetTicksPerSecond, 0);
        rig.assertNoFeed();
    }

    @Test public void feasibleShotFeedsOnlyAfterRealTwoWheelConvergenceAndCompletesAtTimedBoundary() {
        Shooter.useSotm.set(false);
        rig = new RobotRig();
        rig.step(1);
        ShooterCommands.ShotCommand command = ShooterCommands.shootAllPollen(rig.robot);
        command.start();
        command.loop();
        assertEquals(WAITING_FOR_SPEED, command.outcome());
        assertFalse(rig.robot.shooter.upToSpeed());
        rig.assertNoFeed();
        int ticks = 0;
        while (!rig.robot.shooter.upToSpeed() && ticks++ < 60) {
            rig.step(1);
            command.loop();
        }
        assertTrue("synthetic real-controller pair must converge before spinup timeout",
                rig.robot.shooter.upToSpeed());
        assertEquals(FEEDING, command.outcome());
        assertFalse(command.attempted());
        long feedStart = simulation.clock.nowNanos();
        rig.step(1);
        assertEquals(Intake.Mode.FULL_POWER, rig.robot.intake.mode());
        assertEquals(Transfer.Mode.SHOOT_ALL, rig.robot.transfer.mode());
        assertTrue("actual intake SDK power must energize", rig.intake.getPower() > 0);
        assertEquals(Constants.Transfer.SHOOT_ALL_POWER, rig.transfer.getPower(), 0);
        simulation.clock.set(feedStart + Constants.Timings.shootingTimeMs * 1_000_000L);
        command.loop();
        assertEquals(FEEDING, command.outcome());
        assertFalse(command.attempted());
        simulation.clock.add(1);
        command.loop();
        assertEquals(ATTEMPTED, command.outcome());
        assertTrue(command.attempted());
        assertFalse(command.status().isRunning());
        command.end(EndCondition.NATURALLY);
        assertEquals("command end cuts intake on the same tick", 0, rig.intake.getPower(), 0);
        assertEquals("command end cuts transfer on the same tick", 0, rig.transfer.getPower(), 0);
        rig.step(10);
        assertEquals(0, rig.intake.getPower(), 0);
        assertEquals(0, rig.transfer.getPower(), 0);
    }
}
