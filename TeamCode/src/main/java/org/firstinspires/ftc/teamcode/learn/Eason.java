package org.firstinspires.ftc.teamcode.learn;

import static org.firstinspires.ftc.teamcode.util.math.MathUtils.DEG_TO_RAD;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.rmsftc.utils.DriveVector;
import com.rmsftc.utils.ftc.hardware.drivetrain.MecanumDrive;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp
public class Eason extends OpMode {

    DcMotor frontLeft, backLeft, frontRight, backRight;
    IMU imu;

    @Override
    public void init() {
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP
        )));
        imu.resetYaw();

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    @Override
    public void loop() {
        if (gamepad1.back) imu.resetYaw();

        DriveVector driveVector = new DriveVector(
                -gamepad1.left_stick_y,
                gamepad1.left_stick_x,
                gamepad1.right_stick_x
        );

        double robotHeadingRadians = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        driveFieldCentric(driveVector, robotHeadingRadians);

        telemetry.addData("IMU Heading","%2f" ,robotHeadingRadians/DEG_TO_RAD);
        telemetry.update();
    }

    public void driveFieldCentric(DriveVector vector, double botHeading) {
        DriveVector fcVector = vector.fieldCentric(botHeading * DEG_TO_RAD);
        fcVector.scale(0.7);

        frontLeft.setPower(fcVector.forward() + vector.strafe() + vector.turn());
        frontRight.setPower(fcVector.forward() - vector.strafe() - vector.turn());
        backRight.setPower(fcVector.forward() + vector.strafe() - vector.turn());
        backLeft.setPower(fcVector.forward() - vector.strafe() + vector.turn());
    }
}
