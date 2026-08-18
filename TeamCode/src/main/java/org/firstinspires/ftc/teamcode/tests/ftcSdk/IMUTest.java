package org.firstinspires.ftc.teamcode.tests.ftcSdk;

import com.qualcomm.robotcore.eventloop.opmode.Utility;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.library.hardware.gamepad.GamepadEx;

@Utility(name = "FTC SDK IMU Test")
public class IMUTest extends androidTest {
    @Override
    protected void test(HardwareMap hardwareMap, GamepadEx gamepad1, GamepadEx gamepad2, Telemetry telemetry) {
        IMU imu = Constants.createIMU(hardwareMap);
        double yawBeforeReset = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        showStep(telemetry, "Resetting IMU yaw");
        imu.resetYaw();
        sleep(250);
        double yawAfterReset = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

        telemetry.addData("IMU Name", Constants.currentBot.IMU_HARDWAREMAP_NAME);
        telemetry.addData("Yaw Before Reset (deg)", "%.3f", yawBeforeReset);
        telemetry.addData("Yaw After Reset (deg)", "%.3f", yawAfterReset);
        telemetry.addLine("Rotate the robot by hand and rerun if you want to confirm live yaw changes.");
        telemetry.update();
    }

    @Override
    protected String getTestName() {
        return "FTC SDK IMU Factory Test";
    }
}
