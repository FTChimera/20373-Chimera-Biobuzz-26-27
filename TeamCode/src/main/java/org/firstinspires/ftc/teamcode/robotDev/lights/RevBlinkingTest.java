package org.firstinspires.ftc.teamcode.robotDev.lights;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Utility;

import org.firstinspires.ftc.teamcode.util.hardware.lights.rev.RevBlinkinLedDriver;
import org.firstinspires.ftc.teamcode.util.hardware.lights.rev.RevBlinkinLedDriver.BlinkinPattern;

@Utility
public class RevBlinkingTest extends LinearOpMode {
    RevBlinkinLedDriver revBlinkinLedDriver;


    @Override
    public void runOpMode() {
        revBlinkinLedDriver = RevBlinkinLedDriver.from(hardwareMap, "ledStrip");
        telemetry = FtcDashboard.getInstance().getTelemetry();
        setPattern(BlinkinPattern.BLUE);
        telemetry.addData("Status", "Initialized, Should be Blue");
        telemetry.addData("Number of total Patterns", BlinkinPattern.values().length);
        telemetry.update();
        waitForStart();
        cycle(BlinkinPattern.RED);
        for (BlinkinPattern pattern: BlinkinPattern.values()) {
            cycle(pattern);
        }
    }

    private void t() {
        telemetry.clear();
        telemetry.addData("Pattern", revBlinkinLedDriver.getPattern());
        telemetry.update();
    }

    private void cycle(BlinkinPattern blinkinPattern) {
        setPattern(blinkinPattern);
        t();
        sleep(4000);
    }

    void setPattern(BlinkinPattern pattern) { revBlinkinLedDriver.setPattern(pattern);}
}
