package org.firstinspires.ftc.teamcode.config;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.rmsftc.utils.AllianceColor;
import com.rmsftc.utils.MatchMode;
import com.rmsftc.utils.ftc.telemetry.GamepadMenu;
import com.rmsftc.utils.ftc.telemetry.TelemetryLogger;
import com.rmsftc.utils.telemetry.selection.AnswerChoice;
import com.rmsftc.utils.telemetry.selection.Choices;
import com.rmsftc.utils.telemetry.selection.Prompter;
import com.rmsftc.utils.telemetry.selection.Question;
import com.rmsftc.utils.telemetry.selection.Select;

import org.firstinspires.ftc.teamcode.planners.CommonKt;

import java.util.List;

@TeleOp
public class ConfigOpMode extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        // Prompter menus (dpad up/down to move, A to confirm), defaulting
        // to the values currently in OpModeStorage.
        AllianceColor storedColor = OpModeStorage.allianceColor.get();
        MatchMode storedMatchMode = OpModeStorage.matchMode.get();

        Prompter prompter = new Prompter(new TelemetryLogger(telemetry),
                new Question("Alliance", new Choices(
                        storedColor == AllianceColor.BLUE ? 1 : 0,
                        Select.a("Red"), Select.a("Blue"))),
                new Question("Match Mode", new Choices(
                        storedMatchMode == MatchMode.PLAYOFFS ? 1 : 0,
                        Select.a("Qualifications"), Select.a("Playoffs"))),
                // Explicit reset (Blue, Qualifications, garden pose). This is
                // the only place resetToDefault() is called from a competition
                // OpMode: MainTeleOp used to call it on init, which silently
                // clobbered a saved config to BLUE.
                new Question("Reset to defaults", new Choices(
                        0, Select.a("No"), Select.a("Yes"))));
        GamepadMenu menu = new GamepadMenu(prompter, gamepad1);

        telemetry.addLine("==== CONFIGURATION UTILITY OPMODE (use Gp 1) ====");
        telemetry.addLine("Dpad up/down to choose, A to confirm.");
        telemetry.addLine("Answer all three questions to save; just stop the opmode to not save.");
        telemetry.update();

        // Menu phase: runs during init, before waitForStart.
        while (!isStarted() && !isStopRequested() && !menu.isFinished()) {
            menu.update();
            telemetry.update();
        }

        if (menu.isFinished()) {
            List<AnswerChoice> answers = menu.getAnswers();
            if ("Yes".equals(answers.get(2).name)) {
                OpModeStorage.resetToDefault();
                telemetry.addLine("Reset to defaults: Blue, Qualifications, garden pose.");
                telemetry.update();
                requestOpModeStop();
                return;
            }
            AllianceColor color = "Blue".equals(answers.get(0).name)
                    ? AllianceColor.BLUE : AllianceColor.RED;
            MatchMode matchMode = "Playoffs".equals(answers.get(1).name)
                    ? MatchMode.PLAYOFFS : MatchMode.QUALIFICATIONS;

            OpModeStorage.allianceColor.set(color);
            CommonKt.init(color);
            OpModeStorage.matchMode.set(matchMode);

            telemetry.addData("Saved Alliance Color", color);
            telemetry.addData("Saved Match Mode", matchMode);
            telemetry.update();
        }

        requestOpModeStop();
    }
}
