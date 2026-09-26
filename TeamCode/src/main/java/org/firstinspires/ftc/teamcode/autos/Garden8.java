package org.firstinspires.ftc.teamcode.autos;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.configs.Blackboard;
import org.firstinspires.ftc.teamcode.library.AllianceColor;
import org.firstinspires.ftc.teamcode.library.Timer;
import org.firstinspires.ftc.teamcode.library.game.RobotMode;
import org.firstinspires.ftc.teamcode.library.game.State;
import org.firstinspires.ftc.teamcode.library.planners.PlanBuilder;
import org.firstinspires.ftc.teamcode.mechanisms.Robot;

import java.util.concurrent.TimeUnit;

public class Garden8 extends OpMode {
    Robot robot;
    boolean isStopped=false;
    Timer zeroVelocityTimer = new Timer(TimeUnit.SECONDS);
    double pastTime;
    Command plan;
    boolean parked;
    /**
     * User-defined init method
     * <p>
     * This method will be called once, when the INIT button is pressed.
     */
    @Override
    public void init() {
        Scheduler.reset();
        robot = Constants.createRobot(hardwareMap);
        plan = PlanBuilder.startGardenPose(robot)
                .shootPreloads()
                .intakeGarden()
                .schedule();
        robot.build().periodic(new State(RobotMode.AUTONOMOUS)).schedule();
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

        if ((robot.shouldPark() || !Scheduler.isRunning(plan))&&!parked) {
            robot.moveToPark();
            parked = true;
        }
    }

    @Override
    public void stop() {
        if (!isStopped) robot.stop(true);
        Scheduler.reset();
    }

    protected static void setAllianceColor(AllianceColor color) {
        Blackboard.put(new Blackboard.Item(Blackboard.Caption.ALLIANCE, color));
    }
    @Autonomous(name="Red Garden 8", group = Constants.COMPETITION_GROUP, preselectTeleOp = Constants.TELEOP_NAME)
    public static class redalliance extends Garden8 {
        @Override public void init() {setAllianceColor(AllianceColor.RED);super.init();}
    }
    @Autonomous(name="Blue Garden 8", group = Constants.COMPETITION_GROUP, preselectTeleOp = Constants.TELEOP_NAME)
    public static class bluealliance extends Garden8 {
        @Override public void init() {setAllianceColor(AllianceColor.BLUE);super.init();}
    }
}