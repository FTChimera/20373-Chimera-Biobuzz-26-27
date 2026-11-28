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
import java.util.Objects;

@TeleOp
public class ConfigOpMode extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        // Prompter menus (dpad up/down to move, A to confirm), defaulting
        // to the values currently in OpModeStorage.
        AllianceColor storedColor = OpModeStorage.allianceColor.get();
        MatchMode storedMatchMode = OpModeStorage.matchMode.get();

        Prompter prompter = new Prompter(new TelemetryLogger(telemetry),
                new Question("Reset All To Default", new Choices(
                        1,
                        Select.a("Yes"),
                        Select.a("No")
                )),
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
                new Question("Pose = " + OpModeStorage.pose.get().toString()+ " \n Reset to garden?", new Choices(
                        0, Select.a("No"), Select.a("Yes"))));
        GamepadMenu menu = new GamepadMenu(prompter, gamepad1);

        telemetry.addLine("==== CONFIGURATION UTILITY OPMODE (use Gp 1) ====");
        telemetry.addLine("Dpad up/down to choose, A to confirm.");
        telemetry.addLine("Answer all three questions to save; just stop the opmode to not save.");
        telemetry.update();

        // Menu phase: runs during init, before waitForStart.
        while (!isStarted() && !isStopRequested() && !(menu.isFinished() || isFirstAnswerYes(menu))) {
            menu.update();
            telemetry.update();
        }

        if (menu.isFinished() || Objects.equals(menu.getAnswers().get(0).name, "Yes")) {
            List<AnswerChoice> answers = menu.getAnswers();
            if ("Yes".equals(answers.get(0).name)) {
                OpModeStorage.resetToDefault();
                telemetry.addData("Resetting all to default", "");
                telemetry.update();
                sleep(2000);
                requestOpModeStop();
                return;
            }
            if ("Yes".equals(answers.get(3).name)) {
                OpModeStorage.resetPose();
            }
            AllianceColor color = "Blue".equals(answers.get(1).name)
                    ? AllianceColor.BLUE : AllianceColor.RED;
            MatchMode matchMode = "Playoffs".equals(answers.get(2).name)
                    ? MatchMode.PLAYOFFS : MatchMode.QUALIFICATIONS;

            OpModeStorage.allianceColor.set(color);
            CommonKt.init(color);
            OpModeStorage.matchMode.set(matchMode);

            telemetry.addData("Saved Alliance Color", color);
            telemetry.addData("Saved Match Mode", matchMode);
            telemetry.addData("Saved Pose", OpModeStorage.pose.get().toString());
            telemetry.update();
        }
        waitForStart();
        sleep(20);
        requestOpModeStop();
    }

    private boolean isFirstAnswerYes(GamepadMenu menu) {
        if (menu.getAnswers()==null) return false;
        if (menu.getAnswers().isEmpty()) return false;
        return Objects.equals(menu.getAnswers().get(0).name, "Yes");
    }
}
