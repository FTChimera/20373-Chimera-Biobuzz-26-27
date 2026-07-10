package org.firstinspires.ftc.teamcode.learning.kotlinTest

import com.pedropathing.follower.Follower
import com.pedropathing.geometry.BezierLine
import com.pedropathing.geometry.Pose
import com.pedropathing.ivy.Command
import com.pedropathing.ivy.Scheduler
import com.pedropathing.ivy.commands.Commands
import com.pedropathing.ivy.groups.Groups
import com.pedropathing.ivy.pedro.PedroCommands
import com.pedropathing.paths.PathChain
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.OpMode

import org.firstinspires.ftc.teamcode.Constants

//@Autonomous(name = "Example Ivy Auto (Kotlin)", group = "Examples")
class KtExampleIvyAuto: OpMode() {
    private lateinit var follower: Follower

    companion object {
        val startPose = Pose(
            123.5,
            122.8,
            Math.toRadians(37.5)
        )

        val launchPose = Pose(
            108.0,
            108.0,
            Math.toRadians(45.0)
        )

        val set_1Pose = Pose(
            96.0,
            84.0,
            Math.toRadians(0.0)
        )
    }

    lateinit var launchPath: PathChain
    lateinit var set_1Path: PathChain

    fun buildPaths() {
        launchPath = follower.pathBuilder()
            .addPath(BezierLine(
                startPose, launchPose
            ))
            .setLinearHeadingInterpolation(
                startPose.heading, launchPose.heading // Kotlin allows you to access methods like getValue() or setValue() as if the were properties.
            )
            .build()
        set_1Path = follower.pathBuilder()
            .addPath(BezierLine(
                launchPose, set_1Pose
            ))
            .setLinearHeadingInterpolation(
                launchPose.heading, set_1Pose.heading
            )
            .build()
    }

    override fun init() {
        reset()
        follower = Constants.createFollower(hardwareMap)
        buildPaths()
        follower.setStartingPose(startPose)
    }

    override fun start() {
        schedule(
            infinite(follower::update),
            sequential(
                follow(launchPath),
                follow(set_1Path)
            )
        )
    }

    override fun loop() {
        Scheduler.execute()

        telemetry.addData("X", follower.pose.x)
        telemetry.addData("Y", follower.pose.y)
        telemetry.addData("Heading", Math.toDegrees(follower.pose.heading))
        telemetry.update()
    }

    override fun stop() { reset() }

    // Helper methods
    fun schedule(vararg commands: Command) { Scheduler.schedule(*commands) }
    fun follow(pathChain: PathChain) : Command { return PedroCommands.follow(follower, pathChain) }
    fun reset() { Scheduler.reset() }
    fun infinite(run: Runnable) : Command { return Commands.infinite(run) }
    fun sequential(vararg commands: Command) : Command { return Groups.sequential(*commands) }
}