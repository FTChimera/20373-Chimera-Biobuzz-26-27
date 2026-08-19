package org.firstinspires.ftc.teamcode.configs;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Utility;


@Utility(name = "Reset Pose", description = "Configuration")
public class ZeroPose extends ConfigOpMode {

    @Override
    protected Runnable getRunnable() {
        return () -> {
            if (Blackboard.get(Blackboard.Caption.POSE) != null) {
                Blackboard.get(Blackboard.Caption.POSE).reset();
            } else {
                Blackboard.put(Blackboard.Caption.POSE, new Pose());
            }
        };
    }
}
