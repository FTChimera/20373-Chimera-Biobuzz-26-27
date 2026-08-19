package org.firstinspires.ftc.teamcode.configs;

import com.qualcomm.robotcore.eventloop.opmode.Utility;

@Utility(name = "Reset All Blackboard Preferences", description = "Configuration")
public class ResetAllBlackboard extends ConfigOpMode {
    @Override
    protected Runnable getRunnable() {
        return Blackboard::resetAll;
    }
}
