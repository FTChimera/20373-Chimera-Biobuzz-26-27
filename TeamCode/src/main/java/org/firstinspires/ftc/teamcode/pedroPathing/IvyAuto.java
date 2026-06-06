package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.CommandBuilder;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.ivy.commands.Commands;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;



/**
 * An abstract Ivy auton class that allows you to
 * easily create an autonomous by just defining the commands
 * you want to run in each phase of the auton.
 * <p>
 * To use this class, simply extend it and implement the three abstract methods to return the commands you want to run in each phase of the auton.
 * <p>
 * <p>
 * The getInitCommands method will be called once when the INIT button is pressed, and should return the commands you want to run during the init phase of the auton.
 * <p>
 * The getLoopCommands method will be called once when the START button is pressed, and should return the commands you want to run during the main loop of the auton.
 * <p>
 * The getStopCommands method will be called once when the STOP button is pressed, and should return the commands you want to run during the stop phase of the auton.
 */
public abstract class IvyAuto extends LinearOpMode {
    protected abstract Command[] getInitCommands();
    protected abstract Command[] getLoopCommands();
    protected abstract Command[] getStopCommands();

    @Override
    public final void runOpMode() throws InterruptedException {
        Scheduler.reset();
        Scheduler.schedule(getInitCommands());
        while (opModeInInit()) {
            Scheduler.execute();
            telemetry.update();
        }
        waitForStart();
        Scheduler.reset();
        Scheduler.schedule(getLoopCommands());
        while (opModeIsActive()) {
            Scheduler.execute();
            telemetry.update();
        }
        Scheduler.reset();
        Scheduler.schedule(getStopCommands());
        Scheduler.execute();
        telemetry.update();
    }

    protected Command telemetryAddData(String caption, Object data) {
        return Commands.instant(() -> telemetry.addData(caption, data)).requiring(this);
    }

}
