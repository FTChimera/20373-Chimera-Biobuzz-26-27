package org.firstinspires.ftc.teamcode.pedro.commands;

import com.pedropathing.ivy.Command;

import org.firstinspires.ftc.teamcode.util.Timer;

import java.util.concurrent.TimeUnit;

public class TimedScheduler extends Scheduler {
    Timer loopTimer = new Timer(TimeUnit.MILLISECONDS);
    double msInterval;

    public TimedScheduler(double msInterval, Command... commands) {
        super(commands);
        this.msInterval = msInterval;
    }

    @Override
    public void execute() {
        if (loopTimer.get() >= msInterval) {
            super.execute();
            loopTimer.reset();
        }
    }
}
