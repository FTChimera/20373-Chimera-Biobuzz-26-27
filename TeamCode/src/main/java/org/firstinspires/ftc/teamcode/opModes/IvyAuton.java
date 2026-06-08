package org.firstinspires.ftc.teamcode.opModes;

import static com.pedropathing.ivy.commands.Commands.*;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.groups.Groups;
import com.pedropathing.ivy.pedro.PedroCommands;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.configs.Blackboard;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.library.AllianceColor;
import org.firstinspires.ftc.teamcode.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.pedroPathing.CommandConverter;


public abstract class IvyAuton extends OpMode {
    AllianceColor allianceColor;
    IvyAuton_Paths paths;
    Follower follower;
    Intake intake;
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
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(paths.getStartingPose());
        intake = Constants.createIntake(hardwareMap);
    }

    public void start() {
        Scheduler.schedule(Groups.sequential(
            intake.halfPower(),
            PedroCommands.follow(follower, paths.getPath1(), true),
            intake.fullPower(),
            waitMs(1000),
            intake.stop(),
            CommandConverter.toCommand(this::requestOpModeStop)
        ));
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
        Blackboard.put(Blackboard.Caption.POSE, follower.getPose()); // if 0,0,0 issue still persists put it in the scheduler loop and add a check for 0,0,0
        Scheduler.reset();
    }

    public static abstract class IvyAuton_Paths {
        // for variants with multiple paths
         protected abstract PathChain getPath1();
         protected abstract Pose getStartingPose();
    }
}
