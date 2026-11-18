package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.opmodes.MainTeleOp.NAME;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.config.OpModeStorage;
import org.firstinspires.ftc.teamcode.mechanisms.Intake;
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


        waitForStart();
        robot.start();
        //robot.intake.setFullPower();

        while (opModeIsActive()) {
            controls.update();

            if (controls.intakeEject().risingEdge()) {
                robot.intakeEject();
            }
            // Separate ifs (not else-if): the toggle edges are one-cycle
            // pulses, so chaining them under the eject check would drop a
            // press made on the same cycle as an eject press.
            if (controls.intakeControl().risingEdge()) {
                robot.setSmartIntakeControl(true);
            }
            if (controls.intakeControl().fallingEdge()) {
                robot.setSmartIntakeControl(false);
                robot.intake.stop();
            }
            if (controls.deployer().risingEdge()) {
                robot.intake.setDeployed(true);
            }
            if (controls.deployer().fallingEdge()) {
                robot.intake.setDeployed(false);
            }
            // Restore the intake to the mode the toggle button selects:
            // full power if the toggle is on, stopped if it is off.
            if (controls.intakeEject().fallingEdge()) robot.restoreIntake(
                    controls.intakeControl().isPressed()
                            ? Intake.Mode.FULL_POWER
                            : Intake.Mode.IDLE);

            if (controls.panicShoot().isPressed()) {
                if (controls.panicShoot().risingEdge()) robot.panic();
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

                if ((controls.runTransferFast().fallingEdge()
                        || controls.runTransferSlow().fallingEdge()
                        || controls.autoAlignAndStartShoot().fallingEdge())
                        && !controls.runTransferFast().isPressed()
                        && !controls.runTransferSlow().isPressed()
                        && !controls.autoAlignAndStartShoot().isPressed()
                        && robot.transfer.mode() != Transfer.Mode.REVERSE_EJECT) {
                    robot.transfer.stop();
                }

                if (controls.panicShoot().fallingEdge()) robot.endPanicShoot();
            }

            // Y (auto-align) also spins the shooter up while held, so Y
            // alone can shoot; A is the latching toggle for the same.
            robot.setSmartShooterControl(
                    controls.keepShooterOn().isPressed()
                            || controls.autoAlignAndStartShoot().isPressed()
            );
            if (controls.keepShooterOn().fallingEdge()
                    && !controls.autoAlignAndStartShoot().isPressed()) robot.shooter.setVelocity(0, 0);
            if (controls.autoAlignAndStartShoot().fallingEdge()
                    && !controls.keepShooterOn().isPressed()) robot.shooter.setVelocity(0, 0);

            robot.setTeleOpDrive(
                    new DriveVector(
                            -controls.primary().left_stick_y,
                            -controls.primary().left_stick_x,
                            -controls.primary().right_stick_x
                    ),
                    true, false
            );
            if (controls.autoAlignAndStartShoot().isPressed()) { robot.setAutoAlign(); }

            if (controls.stopPark().risingEdge()) {
                robot.cancelPark();
                parkingRequested = false;
                parkingInitiated = false;
                automaticParkingCancelled = true;
            } else if (controls.moveToParkZone().risingEdge()) {
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
            robot.telemetry.addData(
                    "Parking",
                    "requested=%s started=%s auto cancelled=%s",
                    parkingRequested,
                    parkingInitiated,
                    automaticParkingCancelled
            );

            if (robot.opModeTimeRemaining() <=0) {
                robot.stopEverything();
                OpModeStorage.endMatch();
                requestOpModeStop();
                break;
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
