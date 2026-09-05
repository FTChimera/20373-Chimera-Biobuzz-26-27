package org.firstinspires.ftc.teamcode.autos;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.library.Timer;
import org.firstinspires.ftc.teamcode.library.game.RobotMode;
import org.firstinspires.ftc.teamcode.library.planners.PlanBuilder;
import org.firstinspires.ftc.teamcode.library.planners.paths.FromTestPose;
import org.firstinspires.ftc.teamcode.library.planners.paths.Utils;
import org.firstinspires.ftc.teamcode.mechanisms.Robot;

import java.util.concurrent.TimeUnit;

@Autonomous
public class AutoTest extends OpMode {
    Robot robot;
    boolean isStopped=false;
    Timer zeroVelocityTimer = new Timer(TimeUnit.SECONDS);
    double pastTime;
    Command plan;
    /**
     * User-defined init method
     * <p>
     * This method will be called once, when the INIT button is pressed.
     */
    @Override
    public void init() {
        Scheduler.reset();
        robot = Constants.createRobot(hardwareMap);
        PlanBuilder planBuilder = new PlanBuilder(robot, Constants.common.score1red);
        planBuilder.add(new FromTestPose.ToIntakeThreeTest(robot, planBuilder.planner().last()));
        planBuilder.add(new Utils.Wait(planBuilder.planner().last(), 1000));
        planBuilder.add(new FromTestPose.ToIntakeThreeTest(robot, planBuilder.planner().last()));
        plan = planBuilder.schedule();
    }
    @Override public void start() {robot.start();}
    /**
     * User-defined loop method
     * <p>
     * This method will be called repeatedly during the period between when
     * the play button is pressed and when the OpMode is stopped.
     */
    @Override
    public void loop() {
        Scheduler.execute();

        if (robot.getOpModeTimeSeconds() >= RobotMode.AUTONOMOUS.timeLimit()) {
            if (!isStopped) {
                zeroVelocityTimer.reset();
                pastTime = 2;
                robot.stop(false); // Give it time to slow down before saving
                isStopped = true;
            }
            if (robot.requireDrive().getVelocity().getMagnitude() < 0.2 || zeroVelocityTimer.past(pastTime)) {
                robot.saveEndVals();
                requestOpModeStop();
            }
        }

        if (robot.getOpModeTimeSeconds() >= RobotMode.AUTONOMOUS.timeLimit() - 1 || Scheduler.isRunning(plan)) {
            //todo follow path to park
        }
    }

    @Override
    public void stop() {
        if (!isStopped) robot.stop(true);
        Scheduler.reset();
    }

}
