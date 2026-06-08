package org.firstinspires.ftc.teamcode.learning.kotlinTest

import com.pedropathing.follower.Follower
import com.pedropathing.geometry.Pose
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.Constants

@TeleOp(name = "Kotlin TeleOp", group = "Kotlin")
class KotlinTeleOpTest: LinearOpMode() {
    lateinit var follower: Follower
    override fun runOpMode() {
        follower = Constants.createFollower(hardwareMap)
        follower.setStartingPose(Pose())
        waitForStart()
        follower.startTeleOpDrive()
        while (opModeIsActive()) {
            follower.setTeleOpDrive(-gamepad1.left_stick_y.toDouble(), -gamepad1.left_stick_x.toDouble(), -gamepad1.right_stick_x.toDouble(), false)
            follower.update()
            if (gamepad1.back) {
                follower.heading = 0.0;
            }

            telemetry.addLine("---------------------------")
            telemetry.addLine("FIELD CENTRIC KOTLIN TELEOP")
            telemetry.addLine("---------------------------")
            telemetry.addLine()
            telemetry.addData("Yaw", Math.toDegrees(follower.heading))
            telemetry.addLine("Press BACK to reset Yaw")
        }
    }
}