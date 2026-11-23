package org.firstinspires.ftc.teamcode.robotDev.shooter;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Utility;

import org.firstinspires.ftc.teamcode.config.OpModeStorage;
import org.firstinspires.ftc.teamcode.mechanisms.Robot;
import com.rmsftc.utils.RobotMode;
import com.rmsftc.utils.DriveVector;
import org.firstinspires.ftc.teamcode.util.hardware.gamepad.FancyButton;
import org.firstinspires.ftc.teamcode.util.hardware.gamepad.GpFancyButton;
import org.firstinspires.ftc.teamcode.util.shotcalc.ShotContext;
import org.firstinspires.ftc.teamcode.util.shotcalc.TableShotCalc;

import java.util.ArrayList;

/**
 * Shot-table collector, like PoseCollector but for (distance, velocity)
 * table entries. Drive to a distance from the hive, set the flywheel
 * velocity on FTC Dashboard (both flywheels spin at that same speed),
 * hold X to feed a ball and shoot it, and when the velocity scores
 * from that distance, press A to record the entry. Press B to delete
 * the last entry. Entries are shown as ready-to-paste
 * {@code new TableEntry(distance, velocity)} lines for BOTH shooterMap
 * and counterRollerMap in TableShotCalc (the two flywheels share one
 * velocity here, so one entry covers both maps).
 */
@Config
@Utility(
        name = "Shot Table Collector",
        description = "Collect (distance, velocity) shot-table entries: drive, set velocity on Dashboard, hold X to shoot, A to record"
)
public class ShotTableCollector extends LinearOpMode {
    /** Flywheel velocity in ticks/sec, set on FTC Dashboard. Both flywheels spin at this speed. */
    public static int FLYWHEEL_TICKS_PER_SECOND = 1000;

    private Robot robot;
    private final ArrayList<TableShotCalc.TableEntry> entries = new ArrayList<>();
    private GpFancyButton feedButton;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        if (!OpModeStorage.initialized.get()) {
            OpModeStorage.resetToDefault();
        }
        robot = new Robot(telemetry, hardwareMap, OpModeStorage.allianceColor.get(), RobotMode.TELEOP, OpModeStorage.pose.get());
        robot.matchMode = OpModeStorage.matchMode.get();
        robot.debug = false;
        // The dashboard velocity drives the shooter directly; the shot
        // calculator must not override it.
        robot.setSmartShooterControl(false);
        // The X button drives the intake directly; smart intake must not
        // override it.
        robot.setSmartIntakeControl(false);

        // Hold X to feed: intake + transfer run while held, stop on release.
        feedButton = new GpFancyButton(FancyButton.PressType.Normal, () -> gamepad1.x);
        feedButton.declareActionOnPress(button -> {
            if (button.isPressed()) {
                robot.intake.setFullPower();
                robot.transfer.shootAll();
            } else {
                robot.intake.stop();
                robot.transfer.stop();
            }
        });

        telemetry.addLine("Drive with left stick, turn with right stick (slowed for fine positioning).");
        telemetry.addLine("Set FLYWHEEL_TICKS_PER_SECOND on FTC Dashboard (both wheels, ticks/sec).");
        telemetry.addLine("Hold X to shoot. Press A to record the entry, B to delete the last.");
        telemetry.update();

        waitForStart();
        robot.start();

        while (opModeIsActive()) {
            // Drive (field-centric, sticks halved for fine positioning;
            // setTeleOpDrive scales by 0.8 on top of that).
            robot.setTeleOpDrive(
                    new DriveVector(
                            -gamepad1.left_stick_y * 0.5,
                            -gamepad1.left_stick_x * 0.5,
                            -gamepad1.right_stick_x * 0.5),
                    true, false);

            // Both flywheels at the dashboard velocity.
            robot.shooter.setVelocity(FLYWHEEL_TICKS_PER_SECOND, FLYWHEEL_TICKS_PER_SECOND);

            feedButton.update();

            // Applies the drive, updates the pose/vision, and runs the
            // shooter controller toward the dashboard velocity.
            robot.update();

            // Distance to the hive goal for the current hive state
            // (vision infers the state each loop).
            ShotContext ctx = new ShotContext();
            ctx.robotPose = robot.getPose();
            ctx.hiveState = robot.field.getHiveState();
            ctx.allianceColor = OpModeStorage.allianceColor.get();
            ctx.sotm = false;
            // Not shoot-on-move: the release pose is the current pose,
            // so the latency is irrelevant.
            ctx.dt = 0.0;
            double distance = ctx.distanceToTarget();

            if (gamepad1.aWasPressed()) {
                // Round the distance to the nearest 0.5 in for a clean table.
                double roundedDistance = Math.round(distance * 2.0) / 2.0;
                entries.add(new TableShotCalc.TableEntry(roundedDistance, FLYWHEEL_TICKS_PER_SECOND));
            }
            if (gamepad1.bWasPressed() && !entries.isEmpty()) {
                entries.remove(entries.size() - 1);
            }

            telemetry.addData("Distance to hive (in)", "%.1f", distance);
            telemetry.addData("Hive state", robot.field.getHiveState());
            telemetry.addData("Flywheel target (ticks/s)", FLYWHEEL_TICKS_PER_SECOND);
            telemetry.addData("Up to speed", robot.shooter.upToSpeed());
            telemetry.addData("Shooter", robot.shooter.getDebugString());
            telemetry.addLine();
            telemetry.addData("Table entries (" + entries.size() + ")", "");
            if (entries.isEmpty()) {
                telemetry.addLine("Empty. Press A to record the current distance + velocity.");
            } else {
                for (int i = 0; i < entries.size(); i++) {
                    TableShotCalc.TableEntry entry = entries.get(i);
                    telemetry.addData("Entry " + (i + 1),
                            "new TableEntry(%.1f, %.0f)", entry.distance, entry.flywheelTicksPerSecond);
                }
                telemetry.addLine("Paste into BOTH shooterMap and counterRollerMap in TableShotCalc.");
            }
            telemetry.update();
        }

        robot.stopEverything();
    }
}
