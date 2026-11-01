package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.opmodes.MainTeleOp.NAME;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.config.OpModeStorage;
import org.firstinspires.ftc.teamcode.mechanisms.Robot;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.util.RobotMode;
import org.firstinspires.ftc.teamcode.util.hardware.gamepad.DriveVector;
import org.firstinspires.ftc.teamcode.util.hardware.gamepad.FancyButton;
import org.firstinspires.ftc.teamcode.util.hardware.gamepad.GpFancyButton;

@TeleOp(name = NAME, group = Constants.COMPETITION_GROUP)
public class MainTeleOp extends LinearOpMode {
    public static final String NAME = "Main TeleOp";

    GpFancyButton intakeControl;
    GpFancyButton intakeEject;

    GpFancyButton reZeroCornerNextToLoadingZone;
    GpFancyButton reZeroVision;

    GpFancyButton moveToParkZone;

    GpFancyButton runTransferSlow;
    GpFancyButton runTransferFast;
    GpFancyButton panicShoot;
    GpFancyButton autoAlignAndStartShoot;
    GpFancyButton keepShooterOn;

    GpFancyButton flipHive;

    Robot robot;
    private boolean parkingInitiated;


    @Override
    public void runOpMode() throws InterruptedException {
        // ROBOT
        if (!OpModeStorage.initialized) OpModeStorage.resetToDefault();
        robot = new Robot(telemetry, hardwareMap, OpModeStorage.allianceColor, RobotMode.TELEOP, OpModeStorage.pose);
        robot.matchMode = OpModeStorage.matchMode;
        robot.debug = true;
        parkingInitiated = false;

        robot.setManualBrake(false);
        robot.setSmartIntakeControl(true); // Manually control via toggle gp
        robot.setSmartShooterControl(false);

        // Buttons
        intakeControl = new GpFancyButton(FancyButton.PressType.Toggle, ()->gamepad2.x);
        intakeControl.setPressed(true);
        intakeEject = new GpFancyButton(FancyButton.PressType.Normal, ()->gamepad2.back);

        reZeroCornerNextToLoadingZone = new GpFancyButton(FancyButton.PressType.LongPress, ()->gamepad2.back);
        reZeroCornerNextToLoadingZone.declareActionOnPress((fancyButton) -> {if (fancyButton.startPress()) MainTeleOp.this.robot.reZeroCornerNextToLoadingZone();});
        reZeroVision = new GpFancyButton(FancyButton.PressType.LongPress, ()->gamepad2.start);
        reZeroVision.declareActionOnPress((fancyButton) -> {if (fancyButton.startPress()) MainTeleOp.this.robot.reZeroVision();});

        moveToParkZone = new GpFancyButton(FancyButton.PressType.RisingEdge, ()-> gamepad1.a);

        runTransferFast = new GpFancyButton(FancyButton.PressType.Normal, ()->gamepad2.right_bumper);
        runTransferSlow = new GpFancyButton(FancyButton.PressType.Normal, ()->gamepad2.left_bumper);
        panicShoot = new GpFancyButton(FancyButton.PressType.Normal, ()->gamepad2.b);
        autoAlignAndStartShoot = new GpFancyButton(FancyButton.PressType.Normal, ()->gamepad2.y);
        keepShooterOn = new GpFancyButton(FancyButton.PressType.Toggle, ()->gamepad2.y);
        keepShooterOn.setPressed(false);


        waitForStart();
        robot.start();
        robot.intake.setFullPower();

        while (opModeIsActive()) {
            intakeControl.update();
            intakeEject.update();
            reZeroCornerNextToLoadingZone.update();
            reZeroVision.update();
            moveToParkZone.update();
            runTransferFast.update();
            runTransferSlow.update();
            panicShoot.update();
            autoAlignAndStartShoot.update();
            keepShooterOn.update();

            if (intakeEject.startPress()) {
                robot.intakeEject();
            }
            else if (intakeControl.isPressed() && intakeControl.startPress()) {
                robot.setSmartIntakeControl(true);
            } else if (!intakeControl.isPressed() && intakeControl.startPress()) {
                robot.setSmartIntakeControl(false);
                robot.intake.stop();
            }
            if (intakeEject.endPress()) robot.restoreIntake();

            if (runTransferFast.isPressed() && robot.shooter.upToSpeed()) {
                robot.transfer.shootAll();
            }

            if (runTransferSlow.isPressed()) {
                robot.transfer.slow();
            }

            if (runTransferFast.endPress() || runTransferSlow.endPress()) {
                robot.transfer.stop();
            }

            if (panicShoot.startPress()) {
                robot.transfer.panicShoot();
                //robot.panic();
            } else if (panicShoot.endPress()) {
                //robot.endPanicShoot();
            }

            robot.setSmartShooterControl(keepShooterOn.isPressed());
            if (keepShooterOn.startPress() && !keepShooterOn.isPressed()) robot.shooter.setVelocity(0);

            robot.setTeleOpDrive(
                    new DriveVector(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x
                    ),
                    true, false
            );

            if (!parkingInitiated && (robot.shouldPark() || moveToParkZone.isPressed())) {
                robot.moveToParkZone();
                parkingInitiated = true;
            }

            if (robot.opModeTimeRemaining() <=0) {
                robot.stopAllSubsystems();
                OpModeStorage.endMatch();
                requestOpModeStop();
            }


            robot.update();
            robot.telemetry.update();
        }

    }
}
