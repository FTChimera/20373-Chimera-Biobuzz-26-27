package org.firstinspires.ftc.teamcode.configs;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public abstract class ConfigOpMode extends OpMode {
    protected abstract Runnable getRunnable();

    public void loop() {
        requestOpModeStop();
    }
    public void init() {

    }
    public void start() {
        getRunnable().run();
    }
}
