package org.firstinspires.ftc.teamcode.tests.ftcSdk;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.library.DriveVector;
import org.firstinspires.ftc.teamcode.mechanisms.MecanumDrive;

@TeleOp(name = "FTC SDK MecanumDrive Test", group = "1: Tests")
public class MecanumDriveFactoryTest extends androidTest {
    @Override
    protected void test(HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2, Telemetry telemetry) {
        MecanumDrive drive = Constants.createTeleOpFollower(hardwareMap);
        drive.setStartingPose(Constants.RED_DEFAULT_START_POSE);
        drive.startTeleOpDrive();
        drive.setTeleOpDrive(new DriveVector(0.15, 0, 0), false);
        for (int i = 0; i < 6; i++) {
            drive.periodic();
            sleep(100);
        }
        drive.startTeleOpDrive();
        drive.periodic();

        telemetry.addData("Manual Drive Mode", drive.useManualDrive);
        telemetry.addData("Pose", drive.getPose());
        telemetry.addData("Velocity", drive.getVelocity());
        telemetry.addLine("This validates the public teleop drive factory path.");
        telemetry.update();
    }

    @Override
    protected String getTestName() {
        return "FTC SDK MecanumDrive Factory Test";
    }
}
