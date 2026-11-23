package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.rmsftc.cmdlib.cmd.Command;
import com.rmsftc.ftc.ArchitectureManager;
import com.rmsftc.ftc.ArchitecturalOpMode;

import org.firstinspires.ftc.teamcode.config.OpModeStorage;
import org.firstinspires.ftc.teamcode.mechanisms.Robot;
import org.firstinspires.ftc.teamcode.opmodes.MainTeleOp;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.planners.Plan;
import org.firstinspires.ftc.teamcode.planners.Position;
import org.firstinspires.ftc.teamcode.planners.StartPoses;
import org.firstinspires.ftc.teamcode.planners.modules.FromShootGardenPose;
import org.firstinspires.ftc.teamcode.planners.modules.FromStartGardenPose;
import com.rmsftc.utils.RobotMode;

@Autonomous(name = "Garden 8 Ball Autonomous", group = Constants.COMPETITION_GROUP, preselectTeleOp = MainTeleOp.NAME)
public class Garden8 extends ArchitecturalOpMode {
    Robot robot;
    boolean isStopped = false;
    Command plan;
    boolean startParked;
    private boolean started;
    private double lastParkAttemptTimeSec = Double.NEGATIVE_INFINITY;
    private static final double PARK_RETRY_INTERVAL_SEC = 0.5;

    @Override
    protected void onInit() {
        OpModeStorage.pose.set(StartPoses.gardenArea);
        robot = new Robot(telemetry, hardwareMap, OpModeStorage.allianceColor.get(), RobotMode.AUTONOMOUS, OpModeStorage.pose.get());
        robot.matchMode = OpModeStorage.matchMode.get();
        robot.debug = true;
        robot.setSmartShooterControl(true);
    }

    @Override
    protected void build(ArchitectureManager manager) {
        plan = Plan.sequence(Position.GARDEN_START, StartPoses.gardenArea,
                new FromStartGardenPose.ShootPreloads(robot),
                new FromShootGardenPose.ToIntakeGarden(robot));
        manager.schedule(plan);
    }

    @Override
    protected void onLoop() {
        // ArchitecturalOpMode has no start() hook: robot.start() (match
        // timer, lights, vision) runs on the first loop tick, before the
        // first robot.update() that reads the timer.
        if (!started) {
            robot.start();
            started = true;
        }
        if (robot.opModeTimeRemaining() <= 0 || robot.inParkZone()) {
            stopAutonomous();
            return;
        }

        robot.update();
        telemetry.addData("Garden plan", plan == null ? "not scheduled" : manager.scheduler().isScheduled(plan));
        telemetry.addData("Garden follower mode", robot.follower.mode());
        telemetry.addData("Garden follower path", robot.follower.pathIndex());

        if ((robot.shouldPark() || (plan != null && !manager.scheduler().isScheduled(plan))) && !startParked) {
            double now = robot.getOpModeTimeSeconds();
            if (now - lastParkAttemptTimeSec >= PARK_RETRY_INTERVAL_SEC) {
                lastParkAttemptTimeSec = now;
                cancelPlan();
                startParked = robot.moveToParkZone();
            }
        }
    }

    @Override
    public void onStop() {
        if (!isStopped) {
            OpModeStorage.setEndVals(robot.getPoseCompensatedForVelocity());
            robot.stopEverything();
            isStopped = true;
        }
        super.onStop();
    }

    private void stopAutonomous() {
        if (isStopped) {
            return;
        }
        OpModeStorage.setEndVals(robot.getPoseCompensatedForVelocity());
        cancelPlan();
        robot.stopEverything();
        isStopped = true;
        interrupt();
    }

    private void cancelPlan() {
        if (plan != null && manager.scheduler().isScheduled(plan)) {
            manager.cancel(plan);
        }
    }
}
