package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.config.OpModeStorage;
import org.firstinspires.ftc.teamcode.mechanisms.Robot;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.util.RobotMode;
import org.firstinspires.ftc.teamcode.util.hardware.gamepad.DriveVector;
import org.firstinspires.ftc.teamcode.util.hardware.gamepad.FancyButton;
import org.firstinspires.ftc.teamcode.util.hardware.gamepad.GpFancyButton;

@TeleOp(name = MainTeleOp.NAME, group = Constants.COMPETITION_GROUP)
public class MainTeleOp extends LinearOpMode {
    Robot robot;

    GpFancyButton intakeControl;
    GpFancyButton intakeEject;

    GpFancyButton reZeroCornerNextToLoadingZone;

    GpFancyButton moveToParkZone;

    public static final String NAME = "Main TeleOp";

    @Override
    public void runOpMode() throws InterruptedException {
        if (!OpModeStorage.initialized) OpModeStorage.resetToDefaultTesting();
        robot = new Robot(telemetry, hardwareMap, OpModeStorage.allianceColor, RobotMode.TELEOP, OpModeStorage.pose);
        robot.matchMode = OpModeStorage.matchMode;
        robot.debug = true;

        robot.setManualBrake(false);
        robot.setSmartIntakeControl(false); // todo: test with this true
        robot.intake.setAutoEject(false); // todo: test with this true


        intakeControl = new GpFancyButton(FancyButton.PressType.Toggle, ()->gamepad1.x);
        intakeControl.setPressed(true);
        intakeEject = new GpFancyButton(FancyButton.PressType.Normal, ()->gamepad1.back);

        reZeroCornerNextToLoadingZone = new GpFancyButton(FancyButton.PressType.LongPress, ()->gamepad2.back);

        moveToParkZone = new GpFancyButton(FancyButton.PressType.RisingEdge, ()-> gamepad2.a);

        waitForStart();
        robot.start();

        while (opModeIsActive()) {
            intakeControl.update();
            intakeEject.update();
            reZeroCornerNextToLoadingZone.update();
            moveToParkZone.update();

            if (!robot.smartIntakeControl) {
                if (intakeControl.isPressed() && intakeControl.startPress()) {
                    robot.intake.setFullPower();
                } else if (!intakeControl.isPressed() && intakeControl.startPress()) {
                    robot.intake.stop();
                }
                if (!robot.intake.isAutoEjectSet()) {
                    if (intakeEject.startPress()) robot.intakeEject();
                    if (intakeEject.endPress()) robot.intake.setMode(robot.intake.prev);
                }
            }

            if (reZeroCornerNextToLoadingZone.isPressed()) {
                robot.reZeroCornerNextToLoadingZone();
            }

            robot.setTeleOpDrive(
                    new DriveVector(
                            -gamepad1.left_stick_y,
                            gamepad1.left_stick_x,
                            gamepad1.right_stick_x
                    ),
                    true, false
            );

            if (robot.shouldPark() || moveToParkZone.isPressed()) {
                robot.moveToParkZone();
            }

            if (robot.opModeTimeRemaining() <=0 ) {
                robot.stopAllSubsystems();
                OpModeStorage.endMatch();
                requestOpModeStop();
            }


            robot.update();
        }
    }

}