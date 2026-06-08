package org.firstinspires.ftc.teamcode.configs;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.library.robot.Driver;

@TeleOp(name = "Set Default Driver", group = "Configuration")
public class SetDefaultDriver extends ConfigOpMode {

    @Override
    protected Runnable getRunnable() {
        return () -> Blackboard.put(new Blackboard.Item(Blackboard.Caption.DRIVER, Driver.DEFAULT));
    }
}
