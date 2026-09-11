package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.commands.Commands;


public class CommandConverter {
    public static Runnable toRunnable(Command command) {
        return () -> Pedro.runIvy(command);
    }
    public static Command toCommand(Runnable runnable) {
        return Commands.instant(runnable);
    }
}
