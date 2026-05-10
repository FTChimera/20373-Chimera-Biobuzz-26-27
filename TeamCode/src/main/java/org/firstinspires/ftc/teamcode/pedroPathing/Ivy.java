package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.behaviors.EndCondition;

public class Ivy {
    public static void run(Command command) {
        command.start();
        while (!command.done()) {
            command.execute();
        }
        command.end(EndCondition.NATURALLY);
    }
}
