package org.firstinspires.ftc.teamcode.testing;

import static org.junit.Assert.assertEquals;

import com.rmsftc.utils.AllianceColor;
import com.rmsftc.utils.RobotMode;
import com.rmsftc.utils.math.Pose;
import com.pedropathing.api.Paths;

import org.firstinspires.ftc.teamcode.mechanisms.Robot;
import org.firstinspires.ftc.teamcode.mechanisms.Shooter;
import org.firstinspires.ftc.teamcode.pedro.Follower;
import org.firstinspires.ftc.teamcode.util.game.Field;
import org.firstinspires.ftc.teamcode.util.control.DoubleFlywheelFullVelocityController;
import org.junit.Rule;
import org.junit.Test;

import static org.junit.Assert.assertTrue;
import static org.junit.Assert.assertNotNull;

public class RobotTestRigTest {
    @Rule public final FtcSimulationRule simulation = new FtcSimulationRule();

    @Test
    public void sharedBatteryReachesSdkMappingRobotCacheAndBothRealControllers() {
        double[] voltage = {12};
        try (RobotTestRig rig = new RobotTestRig(simulation.clock, () -> voltage[0], 81.5, 65.4, 4)) {
            Robot robot = rig.createRobot(new RecordingTelemetry(), AllianceColor.RED,
                    RobotMode.TELEOP, new Pose(36, 36, 0));
            robot.start();
            assertEquals(1, rig.hardwareMap.voltageSensor.size());
            assertEquals(12, robot.getBatteryVoltage(), 0);
            robot.shooter.setVelocity(1000, 800);
            assertVoltageScale(robot, 1.0 / 12);

            voltage[0] = 9.6;
            rig.advance(0.02);
            simulation.clock.add(20_000_000);
            robot.update();
            assertEquals("Robot battery reads are intentionally cached", 12, robot.getBatteryVoltage(), 0);
            assertVoltageScale(robot, 1.0 / 9.6);

            simulation.clock.add(320_000_000);
            robot.update();
            assertEquals(9.6, robot.getBatteryVoltage(), 0);
            assertVoltageScale(robot, 1.0 / 9.6);
        }
    }

    private static void assertVoltageScale(Robot robot, double expected) {
        assertEquals(expected, ((DoubleFlywheelFullVelocityController)
                robot.shooter.mainMotor.velocityController).getAppliedVoltageScale(), 1e-12);
        assertEquals(expected, ((DoubleFlywheelFullVelocityController)
                robot.shooter.counterRoller.velocityController).getAppliedVoltageScale(), 1e-12);
    }

    @Test
    public void shootOnMoveFacesHiveWithoutCancellingRealPathTranslation() {
        Shooter.useSotm.set(true);
        Pose goal = Field.HiveState.UNTIPPED.goalPose(AllianceColor.RED);
        Pose start = new Pose(goal.x() - 40, goal.y() - 40, 0);
        Pose end = new Pose(start.x(), start.y() + 40, 0);
        try (RobotTestRig rig = new RobotTestRig(simulation.clock)) {
            Robot robot = rig.createRobot(new RecordingTelemetry(), AllianceColor.RED,
                    RobotMode.AUTONOMOUS, start);
            robot.setAutoMaxPathSpeed(0.25);
            robot.setSmartShooterControl(true);
            robot.start();
            robot.follower.follow(Paths.line(start, end).constant(0));
            long pathRun = robot.follower.pathRunId();
            boolean observedCompensation = false;
            for (int i = 0; i < 50; i++) {
                rig.advance(0.02);
                simulation.clock.add(20_000_000);
                robot.setAutoAlign();
                robot.update();
                assertEquals("Aiming must not replace the path with manual/hold control",
                        Follower.Mode.FOLLOW, robot.follower.mode());
                assertEquals(pathRun, robot.follower.pathRunId());
                assertNotNull(robot.follower.headingOverride);
                if (robot.getVelocity().toVector2D().magnitude() > 3) {
                    Pose target = robot.field.getHiveState().goalPose(AllianceColor.RED);
                    Pose release = robot.autoAimerCalculator.getCorrectedPose();
                    double raw = Math.atan2(target.y() - release.y(), target.x() - release.x());
                    double compensated = robot.autoAimerCalculator.getTargetHeading();
                    double lead = Math.atan2(Math.sin(compensated - raw), Math.cos(compensated - raw));
                    observedCompensation |= Math.abs(lead) > 0.01;
                    assertTrue(robot.shooter.hasNonzeroVelocityTarget());
                }
            }
            assertTrue("Real drive output must move measured odometry: distance=" + rig.distanceTravelled()
                    + " pose=" + robot.getPose() + " follower=" + robot.follower.debug(),
                    rig.distanceTravelled() > 5);
            assertTrue("Real Foresight must retain and advance the running path", robot.follower.completion() > 0.1);
            assertTrue("Measured chassis motion must change the launch heading", observedCompensation);
        }
    }
}
