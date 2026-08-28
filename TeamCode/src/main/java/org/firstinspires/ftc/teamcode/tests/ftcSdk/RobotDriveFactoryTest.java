package org.firstinspires.ftc.teamcode.tests.ftcSdk;

import com.qualcomm.robotcore.eventloop.opmode.Utility;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.library.game.State;
import org.firstinspires.ftc.teamcode.library.hardware.gamepad.Button;
import org.firstinspires.ftc.teamcode.library.hardware.gamepad.GamepadEx;
import org.firstinspires.ftc.teamcode.library.hardware.gamepad.GpBindings;
import org.firstinspires.ftc.teamcode.mechanisms.RobotDrive;

@Utility(name = "FTC SDK RobotDrive Test")
public class RobotDriveFactoryTest extends androidTest {
    @Override
    protected void test(HardwareMap hardwareMap, GamepadEx gamepad1, GamepadEx gamepad2, Telemetry telemetry) {
        State GpBindings = new State(gamepad1.result());
        RobotDrive drive = Constants.createDrive(hardwareMap);
        drive.setStartingPose(Constants.RED_DEFAULT_START_POSE);
        drive.startTeleOpDrive();
        drive.periodic(GpBindings);
        for (int i = 0; i < 6; i++) {
            drive.periodic(GpBindings);
            sleep(100);
        }
        drive.startTeleOpDrive();
        drive.periodic(GpBindings);

        telemetry.addData("Manual Drive Mode", drive.isUsingManualDrive());
        telemetry.addData("Pose", drive.getPose());
        telemetry.addData("Velocity", drive.getVelocity());
        telemetry.addLine("This validates the public teleop drive factory path.");
        telemetry.update();
    }

    @Override
    protected String getTestName() {
        return "FTC SDK RobotDrive Factory Test";
    }
}
