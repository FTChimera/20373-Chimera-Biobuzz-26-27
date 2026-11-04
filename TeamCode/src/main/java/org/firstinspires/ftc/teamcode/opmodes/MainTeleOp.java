package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.opmodes.MainTeleOp.NAME;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.config.OpModeStorage;
import org.firstinspires.ftc.teamcode.mechanisms.Robot;
import org.firstinspires.ftc.teamcode.mechanisms.Transfer;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.util.RobotMode;
import org.firstinspires.ftc.teamcode.util.hardware.gamepad.DriveVector;

@TeleOp(name = NAME, group = Constants.COMPETITION_GROUP)
public class MainTeleOp extends LinearOpMode {
    public static final String NAME = "Main TeleOp";

    private Controls controls;
    private Robot robot;
    private boolean parkingInitiated;
    private boolean parkingRequested;
    private boolean automaticParkingCancelled;
    private double lastParkAttemptTimeSec;
    private static final double PARK_RETRY_INTERVAL_SEC = 0.5;

    protected boolean oneController = false;

    @Override
    public void runOpMode() throws InterruptedException {
        // ROBOT
        if (!OpModeStorage.initialized) OpModeStorage.resetToDefault();
        robot = new Robot(telemetry, hardwareMap, OpModeStorage.allianceColor, RobotMode.TELEOP, OpModeStorage.pose);
        robot.matchMode = OpModeStorage.matchMode;
        robot.debug = true;
        parkingInitiated = false;
        parkingRequested = false;
        automaticParkingCancelled = false;
        lastParkAttemptTimeSec = Double.NEGATIVE_INFINITY;

        robot.setManualBrake(false);
        robot.setSmartIntakeControl(true); // Manually control via toggle gp
        robot.setSmartShooterControl(false);

        robot.setManualBrake(false);
        robot.setSmartIntakeControl(true); // Manually control via toggle gp
        robot.setSmartShooterControl(false);

        controls = new Controls(gamepad1, gamepad2, oneController);
        controls.reZeroVision().declareActionOnPress((fancyButton) -> {
            if (fancyButton.startPress() && !MainTeleOp.this.robot.reZeroVision()) {
                MainTeleOp.this.robot.telemetry.addLine("Vision re-zero skipped: " + MainTeleOp.this.robot.vision.getLocalizationReliabilityReason());
            }
        });


        waitForStart();
        robot.start();
        robot.intake.setFullPower();

        while (opModeIsActive()) {
            controls.update();

            if (controls.intakeEject().startPress()) {
                robot.intakeEject();
            }
            else if (controls.intakeControl().isPressed() && controls.intakeControl().startPress()) {
                robot.setSmartIntakeControl(true);
            } else if (!controls.intakeControl().isPressed() && controls.intakeControl().startPress()) {
                robot.setSmartIntakeControl(false);
                robot.intake.stop();
            }
            if (controls.intakeEject().endPress()) robot.restoreIntake();

            if (controls.panicShoot().isPressed()) {
                if (controls.panicShoot().startPress()) robot.panic();
            } else {
                if (controls.runTransferFast().isPressed()
                        && robot.shooter.upToSpeed()
                        && robot.transfer.mode() != Transfer.Mode.REVERSE_EJECT) {
                    robot.transfer.shootAll();
                }

                if (controls.runTransferSlow().isPressed()
                        && robot.transfer.mode() != Transfer.Mode.REVERSE_EJECT) {
                    robot.transfer.slow();
                }

                if (controls.autoAlignAndStartShoot().isPressed()
                        && robot.shooter.upToSpeed()
                        && robot.transfer.mode() != Transfer.Mode.REVERSE_EJECT) {
                    robot.transfer.shootAll();
                }

                if ((controls.runTransferFast().endPress()
                        || controls.runTransferSlow().endPress()
                        || controls.autoAlignAndStartShoot().endPress())
                        && !controls.runTransferFast().isPressed()
                        && !controls.runTransferSlow().isPressed()
                        && !controls.autoAlignAndStartShoot().isPressed()) {
                    robot.transfer.stop();
                }

                if (controls.panicShoot().endPress()) robot.endPanicShoot();
            }

            robot.setSmartShooterControl(
                    controls.keepShooterOn().isPressed()
            );
            if (controls.keepShooterOn().startPress() && !controls.keepShooterOn().isPressed()) robot.shooter.setVelocity(0);

            robot.setTeleOpDrive(
                    new DriveVector(
                            -controls.primary().left_stick_y,
                            -controls.primary().left_stick_x,
                            -controls.primary().right_stick_x
                    ),
                    true, false
            );
            if (controls.autoAlignAndStartShoot().isPressed()) { robot.setAutoAlign(); }

            if (controls.stopPark().startPress()) {
                robot.cancelPark();
                parkingRequested = false;
                parkingInitiated = false;
                automaticParkingCancelled = true;
            } else if (controls.moveToParkZone().startPress()) {
                parkingRequested = true;
                automaticParkingCancelled = false;
            } else if (!automaticParkingCancelled && robot.shouldPark()) {
                parkingRequested = true;
            }
            if (parkingRequested && !parkingInitiated) {
                double now = robot.getOpModeTimeSeconds();
                if (now - lastParkAttemptTimeSec >= PARK_RETRY_INTERVAL_SEC) {
                    lastParkAttemptTimeSec = now;
                    parkingInitiated = robot.moveToParkZone();
                }
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
    @TeleOp(name = NAME + " (One Controller)", group = Constants.COMPETITION_GROUP)
    public static class MainTeleOneController extends MainTeleOp {
        public MainTeleOneController() { this.oneController = true; }
    }
}
