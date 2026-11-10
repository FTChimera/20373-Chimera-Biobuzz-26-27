package org.firstinspires.ftc.teamcode.robotDev.subsystemTests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Utility;

import org.firstinspires.ftc.teamcode.util.AllianceColor;

@Utility
public class IntakePrototypeMechanismTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        IntakeTestPrototype intake = new IntakeTestPrototype(hardwareMap, AllianceColor.RED);
        intake.allowOverrideOverEject = true;
        telemetry = FtcDashboard.getInstance().getTelemetry();
        telemetry.clear();
        waitForStart();
        intake.setLowPower();
        while (opModeIsActive()) {
            intake.update();
            telemetry.addData("Intake Mode", intake.mode());
            //telemetry.addData("Debug string", intake.getDebugString());
            telemetry.update();
        }
    }
}
