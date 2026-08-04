package org.firstinspires.ftc.teamcode.tests.ftcSdk;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.library.hardware.gamepad.GamepadEx;

@TeleOp(name = "FTC SDK Follower Test", group = "1: Tests")
public class FollowerFactoryTest extends androidTest {
    @Override
    protected void test(HardwareMap hardwareMap, GamepadEx gamepad1, GamepadEx gamepad2, Telemetry telemetry) {
        Follower follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(Constants.RED_DEFAULT_START_POSE);
        follower.update();
        follower.startTeleopDrive();
        follower.setTeleOpDrive(0, 0, 0, false);
        for (int i = 0; i < 5; i++) {
            follower.update();
            sleep(100);
        }

        Pose pose = follower.getPose();
        telemetry.addData("Pose", pose);
        telemetry.addData("Velocity", follower.getVelocity());
        telemetry.addData("Angular Velocity", follower.getAngularVelocity());
        telemetry.addLine("This test should not move the robot.");
        telemetry.update();
    }

    @Override
    protected String getTestName() {
        return "FTC SDK Follower Factory Test";
    }
}
