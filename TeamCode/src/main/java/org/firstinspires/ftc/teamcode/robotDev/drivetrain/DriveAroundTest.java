package org.firstinspires.ftc.teamcode.robotDev.drivetrain;

import com.pedropathing.revhub.drivetrains.Mecanum;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Utility;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.util.hardware.gamepad.DriveVector;

@Utility(name = "Drive around test Robot Centric", description = "Drive around Robot Centric")
public class DriveAroundTest extends OpMode {
    boolean fC;
    Mecanum drivetrain;
    @Override
    public void init() {
        drivetrain = new Mecanum(hardwareMap, Constants.MECANUM_CONFIG);
        drivetrain.stop();
        drivetrain.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        drivetrain.config.manualBrakeMode.set(false);
        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void loop() {
        telemetry.addData("Current Amps", drivetrain.currentAmps());
        drivetrain.drive(
                new DriveVector(
                    -gamepad1.left_stick_y,
                    gamepad1.left_stick_x,
                    gamepad1.right_stick_x
                ),
                true
        );
        drivetrain.debug().forEach(telemetry::addData);
        telemetry.update();
    }
}
