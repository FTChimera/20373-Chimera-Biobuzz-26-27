package org.firstinspires.ftc.teamcode.learning.teachEason;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.ivy.commands.Commands;
import com.pedropathing.ivy.pedro.PedroCommands;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Constants;

@Autonomous
public class IvyTest extends LinearOpMode {

    Pose startPose = new Pose(86.59, 9.18, Math.toRadians(90));
    Path path1 = new Path(new BezierLine(startPose, new Pose(84, 12, Math.toRadians(65.56))));
    /**
     * Override this method and place your code here.
     * <p>
     * Please do not catch {@link InterruptedException}s that are thrown in your OpMode
     * unless you are doing it to perform some brief cleanup, in which case you must exit
     * immediately afterward. Once the OpMode has been told to stop, your ability to
     * control hardware will be limited.
     *
     * @throws InterruptedException When the OpMode is stopped while calling a method
     *                              that can throw {@link InterruptedException}
     */
    @Override
    public void runOpMode() throws InterruptedException {
        Scheduler.reset();
        path1.setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(65.56));
        Follower follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        waitForStart();
        Command followPath1 = PedroCommands.follow(follower, new PathChain(path1));
        Scheduler.schedule(followPath1, Commands.infinite(follower::update));
        while (opModeIsActive()) {
            Scheduler.execute();
        }
    }
}
