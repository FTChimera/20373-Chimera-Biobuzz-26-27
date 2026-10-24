package org.firstinspires.ftc.teamcode.config;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Utility;

import org.firstinspires.ftc.teamcode.util.AllianceColor;
import org.firstinspires.ftc.teamcode.util.MatchMode;

@Utility
public class ConfigOpMode extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addLine("==== CONFIGURATION UTILITY OPMODE (use Gp 1) ====");
        telemetry.addData("Controls", "Press A for Red Alliance, B for Blue Alliance");
        telemetry.addLine("Press Back to save and end, just stop the opmode to not save");
        telemetry.addLine();
        telemetry.update();
        waitForStart();
        AllianceColor color = OpModeStorage.allianceColor;
        MatchMode matchMode = OpModeStorage.matchMode;
        while (opModeIsActive()) {
            telemetry.addLine("==== CONFIGURATION UTILITY OPMODE (use Gp 1) ====");
            telemetry.addData("Controls", "Press A for Red Alliance, B for Blue Alliance");
            telemetry.addLine("Press X for Qualifications, Y for Playoffs");
            telemetry.addLine("Press Back to save and end, just stop the opmode to not save");
            telemetry.addLine();
            if (gamepad1.aWasPressed()) {
                color = AllianceColor.RED;
            }
            if (gamepad1.bWasPressed()) {
                color = AllianceColor.BLUE;
            }
            if (gamepad1.xWasPressed()) {
                matchMode = MatchMode.QUALIFICATIONS;
            }
            if (gamepad1.yWasPressed()) {
                matchMode = MatchMode.PLAYOFFS;
            }
            if (gamepad1.backWasPressed()) {
                OpModeStorage.allianceColor = color;
                OpModeStorage.matchMode = matchMode;
                requestOpModeStop();
            }
            telemetry.addData("Current Alliance Color", color);
            telemetry.addData("Current Match Mode", matchMode);
        }
    }
}
