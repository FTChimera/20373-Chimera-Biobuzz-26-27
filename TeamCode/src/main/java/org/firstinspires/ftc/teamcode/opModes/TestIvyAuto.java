package org.firstinspires.ftc.teamcode.opModes;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.commands.Commands;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.pedroPathing.IvyAuto;

@Autonomous(name = "TEST IVY AUTO", group = "1: Autonomous")
public class TestIvyAuto extends IvyAuto {
    @Override
    protected Command[] getInitCommands() {
        return new Command[]{
                telemetryAddData("Status", "Init")
        };
    }

    @Override
    protected Command[] getLoopCommands() {
        return new Command[]{
                Commands.infinite(() -> telemetryAddData("Status", "Running"))
        };
    }

    @Override
    protected Command[] getStopCommands() {
        return new Command[]{
                Command.NOOP
        };
    }
}
