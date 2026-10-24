package org.firstinspires.ftc.teamcode.robotDev.drivetrain;

import com.pedropathing.follower.ManualDrive;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Utility;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.pedro.Follower;
import org.firstinspires.ftc.teamcode.util.hardware.gamepad.DriveVector;
import org.firstinspires.ftc.teamcode.util.poses.PoseFactory;
import org.firstinspires.ftc.teamcode.util.math.MathUtils;


@Utility(name = "Follower Test", description = "Test the follower's localization, and velocities capability")
public class FollowerTest extends OpMode {
    Follower follower;
    /**
     * User-defined init method
     * <p>
     * This method will be called once, when the INIT button is pressed.
     */
    @Override
    public void init() {
        Constants.PINPOINT_CONFIG.resetMode.set(PinpointLocalizer.ResetMode.RESET_AND_RECALIBRATE_IMU);
        follower = Constants.createFollower(hardwareMap);
        follower.holdEnd.set(false);
        follower.localizer.setPose(PoseFactory.zero());
    }

    /**
     * User-defined loop method
     * <p>
     * This method will be called repeatedly during the period between when
     * the play button is pressed and when the OpMode is stopped.
     */
    @Override
    public void loop() {
        follower.manual(
                ManualDrive.fieldCentric(
                        new DriveVector(
                                -gamepad1.left_stick_y,
                                gamepad1.left_stick_x,
                                gamepad1.right_stick_x
                        ),
                        follower.localizer.pose().heading()
                )
        );

        if (gamepad1.back) follower.localizer.setHeading(0);

        follower.update();
        telemetry.addData("=== FOLLOWER DEBUG ===", follower.debug().toString());
        telemetry.addData("Pose", PoseFactory.of(follower.localizer.pose()).toString());
        telemetry.addData("Heading in Degrees", "(%.2f)", follower.localizer.pose().heading()/ MathUtils.DEG_TO_RAD);
        telemetry.addData("Velocity", follower.localizer.velocity());
        telemetry.addData("Twist", follower.localizer.twist());
        telemetry.addData("Mode", follower.mode());
        telemetry.update();
    }
}
