package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;

/**
 * Blackboard - A shared data structure for storing global information
 *
 * This class serves as a central repository for data that needs to be accessed across multiple subsystems and op modes.
 * It can be used to store the robot's current pose, sensor readings, or any other information that should be globally accessible.
 *
 * Key Features:
 * - Static variables for easy access across the codebase
 * - Configurable fields for tuning parameters at runtime
 *
 */
@Configurable
public class Blackboard {
    public static Pose pose = null;

}
