package org.firstinspires.ftc.teamcode.configs;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Reset All Blackboard Preferences", group = "Configuration")
public class ResetAllBlackboard extends ConfigOpMode {
    @Override
    protected Runnable getRunnable() {
        return Blackboard::resetAll;
    }
}
