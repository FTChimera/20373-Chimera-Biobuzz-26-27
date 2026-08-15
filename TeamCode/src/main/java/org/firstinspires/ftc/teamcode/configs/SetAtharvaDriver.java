package org.firstinspires.ftc.teamcode.configs;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.rmsftc.library.robot.Driver;

@TeleOp(name = "Set Atharva Driver", group = "Configuration")
public class SetAtharvaDriver extends ConfigOpMode {

    @Override
    protected Runnable getRunnable() {
        return () -> Blackboard.put(new Blackboard.Item(Blackboard.Caption.DRIVER, Driver.ATHARVA));
    }
}
