package org.firstinspires.ftc.teamcode.tests.ftcSdk;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.library.DriveVector;
import org.firstinspires.ftc.teamcode.library.robot.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.library.hardware.gamepad.GamepadEx;

@TeleOp(name = "FTC SDK Drivetrain Test", group = "1: Tests")
public class DrivetrainFactoryTest extends androidTest {
    private static final double SAFE_POWER = 0.2;

    @Override
    protected void test(HardwareMap hardwareMap, GamepadEx gamepad1, GamepadEx gamepad2, Telemetry telemetry) {
        Drivetrain drivetrain = Constants.createDrivetrain(hardwareMap);
        drivetrain.startTeleOpDrive();

        showStep(telemetry, "Forward at 20% power");
        drivetrain.setTeleOpDrive(new DriveVector(SAFE_POWER, 0, 0), 0);
        sleep(700);

        showStep(telemetry, "Strafe at 20% power");
        drivetrain.setTeleOpDrive(new DriveVector(0, SAFE_POWER, 0), 0);
        sleep(700);

        showStep(telemetry, "Rotate at 20% power");
        drivetrain.setTeleOpDrive(new DriveVector(0, 0, SAFE_POWER), 0);
        sleep(700);

        drivetrain.startTeleOpDrive();
        telemetry.addData("Drive Motors", drivetrain.getDriveMotors().length);
        telemetry.addData("Drive Stack Mechanisms", Constants.getMechanisms());
        telemetry.addLine("Run this with the robot safely off the ground first.");
        telemetry.update();
    }

    @Override
    protected String getTestName() {
        return "FTC SDK Drivetrain Factory Test";
    }
}
