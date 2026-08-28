package org.firstinspires.ftc.teamcode.opModes;

import static com.pedropathing.ivy.commands.Commands.*;

import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.groups.Groups;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.configs.Blackboard;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.library.AllianceColor;
import org.firstinspires.ftc.teamcode.library.game.RobotMode;
import org.firstinspires.ftc.teamcode.library.game.State;
import org.firstinspires.ftc.teamcode.mechanisms.Robot;
import org.firstinspires.ftc.teamcode.library.robot.RobotIvyBuilder;
import org.firstinspires.ftc.teamcode.pedroPathing.CommandConverter;


public abstract class IvyAuton extends OpMode {
    AllianceColor allianceColor;
    IvyAuton_Paths paths;
    Robot robot;
    RobotIvyBuilder commands;
    State autonomousState;
     // = new Path(new BezierLine(
    /*
            new Pose(36, 12, Math.toRadians(180)),
            new Pose(36, 36, Math.toRadians(180))
    ));
     */
    /**
     * User-defined init method
     * <p>
     * This method will be called once, when the INIT button is pressed.
     */

    public IvyAuton(AllianceColor col, IvyAuton_Paths paths) {
        this.allianceColor = col;
        this.paths = paths;
    }

    @Override
    public void init() {
        Scheduler.reset();
        robot = Constants.createRobot(hardwareMap, RobotMode.AUTONOMOUS);
        commands = robot.build();
        autonomousState = new State(RobotMode.AUTONOMOUS);
    }

    public void start() {
        robot.start();
        Scheduler.schedule(
                commands.periodic(autonomousState),
                Groups.sequential(
                    commands.setStartingPose(paths.getStartingPose()),
                    commands.runIntakeHalfPower(),
                    commands.followPath(paths.getPath1(), true),
                    commands.startIntake(),
                    waitMs(1000),
                    commands.stopIntake(),
                    CommandConverter.toCommand(this::requestOpModeStop)
                )
        );
    }
    /**
     * User-defined loop method
     * <p>
     * This method will be called repeatedly during the period between when
     * the play button is pressed and when the OpMode is stopped.
     */
    @Override
    public void loop() {
        Scheduler.execute();
    }

    @Override
    public void stop() {
        robot.stop();
        Scheduler.reset();
    }

    public static abstract class IvyAuton_Paths {
        // for variants with multiple paths
         protected abstract PathChain getPath1();
         protected abstract Pose getStartingPose();
    }
}
