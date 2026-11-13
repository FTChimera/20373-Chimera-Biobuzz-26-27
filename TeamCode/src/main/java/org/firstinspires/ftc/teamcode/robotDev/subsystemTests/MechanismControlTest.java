package org.firstinspires.ftc.teamcode.robotDev.subsystemTests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Utility;

import org.firstinspires.ftc.teamcode.config.OpModeStorage;
import org.firstinspires.ftc.teamcode.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.Shooter;
import org.firstinspires.ftc.teamcode.mechanisms.Transfer;

@Config
@Utility(
        name = "Mechanism Control Test",
        description = "MainTeleOp-style shooter, intake, and transfer controls without drivetrain"
)
public class MechanismControlTest extends LinearOpMode {
    public static int MAIN_SHOOTER_TICKS_PER_SECOND = 1000;
    public static int COUNTER_ROLLER_TICKS_PER_SECOND = 1000;

    @Override
    public void runOpMode() throws InterruptedException {
        if (!OpModeStorage.initialized) {
            OpModeStorage.resetToDefault();
        }

        Intake intake = new Intake(hardwareMap, OpModeStorage.allianceColor);
        Transfer transfer = new Transfer(hardwareMap);
        Shooter shooter = new Shooter(hardwareMap, OpModeStorage.allianceColor);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addLine("Gamepad 2: X intake, Back eject, DPad Left deployer, A shooter");
        telemetry.addLine("RB fast transfer, LB slow transfer, Y fast transfer, B panic shoot");
        telemetry.addLine("Set shooter velocities in FTC Dashboard before enabling A.");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) {
            return;
        }

        boolean smartIntakeControl = true;
        boolean shooterEnabled = false;
        boolean deployerDeployed = false;
        boolean lastIntakeControl = false;
        boolean lastIntakeEject = false;
        boolean lastDeployer = false;
        boolean lastTransferFast = false;
        boolean lastTransferSlow = false;
        boolean lastPanicShoot = false;
        boolean lastAutoShoot = false;
        boolean lastShooterEnable = false;

        intake.setFullPower();

        try {
            while (opModeIsActive()) {
                boolean intakeControl = gamepad2.x;
                boolean intakeEject = gamepad2.back;
                boolean deployer = gamepad2.dpad_left;
                boolean transferFast = gamepad2.right_bumper;
                boolean transferSlow = gamepad2.left_bumper;
                boolean panicShoot = gamepad2.b;
                boolean autoShoot = gamepad2.y;
                boolean shooterEnable = gamepad2.a;

                if (intakeEject && !lastIntakeEject) {
                    intake.forceEject();
                    transfer.reverse();
                }
                if (intakeControl && !lastIntakeControl) {
                    smartIntakeControl = !smartIntakeControl;
                    if (!smartIntakeControl) {
                        intake.stop();
                    }
                }
                if (deployer && !lastDeployer) {
                    deployerDeployed = !deployerDeployed;
                    intake.setDeployed(deployerDeployed);
                }
                if (!intakeEject && lastIntakeEject) {
                    intake.stopEjecting();
                    transfer.stop();
                }

                if (panicShoot) {
                    if (!lastPanicShoot) {
                        shooter.panic();
                        transfer.panicShoot();
                    }
                } else {
                    if (transferFast
                            && shooter.upToSpeed()
                            && transfer.mode() != Transfer.Mode.REVERSE_EJECT) {
                        transfer.shootAll();
                    }
                    if (transferSlow
                            && transfer.mode() != Transfer.Mode.REVERSE_EJECT) {
                        transfer.slow();
                    }
                    if (autoShoot
                            && shooter.upToSpeed()
                            && transfer.mode() != Transfer.Mode.REVERSE_EJECT) {
                        transfer.shootAll();
                    }
                    if ((lastTransferFast && !transferFast
                            || lastTransferSlow && !transferSlow
                            || lastAutoShoot && !autoShoot)
                            && !transferFast
                            && !transferSlow
                            && !autoShoot) {
                        transfer.stop();
                    }
                    if (lastPanicShoot) {
                        shooter.panicking = false;
                        shooter.setVelocity(0, 0);
                        transfer.stop();
                    }
                }

                if (shooterEnable && !lastShooterEnable) {
                    shooterEnabled = !shooterEnabled;
                    if (!shooterEnabled) {
                        shooter.setVelocity(0, 0);
                    }
                }
                if (shooterEnabled && !panicShoot) {
                    shooter.setVelocity(
                            MAIN_SHOOTER_TICKS_PER_SECOND,
                            COUNTER_ROLLER_TICKS_PER_SECOND
                    );
                }

                if (smartIntakeControl && !intake.isEjecting()) {
                    intake.setFullPower();
                }

                intake.update();
                transfer.update();
                shooter.update();

                telemetry.addData("Smart intake", smartIntakeControl);
                telemetry.addData("Shooter enabled", shooterEnabled);
                telemetry.addData(
                        "Shooter targets (main/counter)",
                        "%d / %d ticks/s",
                        MAIN_SHOOTER_TICKS_PER_SECOND,
                        COUNTER_ROLLER_TICKS_PER_SECOND
                );
                telemetry.addData("Intake", intake.getDebugString());
                telemetry.addData("Transfer", transfer.getDebugString());
                telemetry.addData("Shooter", shooter.getDebugString());
                telemetry.update();

                lastIntakeControl = intakeControl;
                lastIntakeEject = intakeEject;
                lastDeployer = deployer;
                lastTransferFast = transferFast;
                lastTransferSlow = transferSlow;
                lastPanicShoot = panicShoot;
                lastAutoShoot = autoShoot;
                lastShooterEnable = shooterEnable;
            }
        } finally {
            intake.stopImmediately();
            transfer.stopImmediately();
            shooter.stopImmediately();
        }
    }
}
