package org.firstinspires.ftc.teamcode.learning.kotlinTest

import com.pedropathing.follower.Follower
import com.pedropathing.geometry.Pose
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.Constants
import com.rmsftc.library.hardware.gamepad.Button
import com.rmsftc.library.hardware.gamepad.GamepadEx

@TeleOp(name = "Kotlin TeleOp", group = "Kotlin")
class KotlinTeleOpTest: LinearOpMode() {
    lateinit var follower: Follower
    override fun runOpMode() {
        follower = Constants.createFollower(hardwareMap)
        val driver = GamepadEx(gamepad1)
        follower.setStartingPose(Pose())
        waitForStart()
        follower.startTeleOpDrive()
        while (opModeIsActive()) {
            driver.update()
            follower.setTeleOpDrive(driver.leftStickY(), -driver.leftStickX(), -driver.rightStickX(), false)
            follower.update()
            if (driver.isPressed(Button.BACK)) {
                follower.heading = 0.0;
            }

            telemetry.addLine("---------------------------")
            telemetry.addLine("FIELD CENTRIC KOTLIN TELEOP")
            telemetry.addLine("---------------------------")
            telemetry.addLine()
            telemetry.addData("Yaw", Math.toDegrees(follower.heading))
            telemetry.addLine("Press BACK to reset Yaw")
            telemetry.update()
        }
    }
}