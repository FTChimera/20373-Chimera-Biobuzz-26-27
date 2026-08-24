package org.firstinspires.ftc.teamcode.configs;

import com.pedropathing.geometry.Pose;
import com.pedropathing.telemetry.SelectableOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.library.AllianceColor;
import org.firstinspires.ftc.teamcode.library.robot.Driver;

import java.util.function.Supplier;

@TeleOp(name = "Configuration", group = "Configuration")
public class ConfigOpMode extends SelectableOpMode {
    public ConfigOpMode() {
        super("Robot Configuration", scope -> {
            scope.folder("Alliance Color", alliance -> {
                alliance.add("Set Alliance Color RED", action("Set Alliance Color RED",
                        () -> Blackboard.put(Blackboard.Caption.ALLIANCE, AllianceColor.RED)));
                alliance.add("Set Alliance Color BLUE", action("Set Alliance Color BLUE",
                        () -> Blackboard.put(Blackboard.Caption.ALLIANCE, AllianceColor.BLUE)));
            });
            scope.folder("Driver", driver -> {
                driver.add("Set Atharva to be Driver", action("Set Atharva to be Driver",
                        () -> Blackboard.put(Blackboard.Caption.DRIVER, Driver.ATHARVA)));
                driver.add("Set Shlok to be Driver", action("Set Shlok to be Driver",
                        () -> Blackboard.put(Blackboard.Caption.DRIVER, Driver.SHLOK)));
                driver.add("Set Eason to be Driver", action("Set Eason to be Driver",
                        () -> Blackboard.put(Blackboard.Caption.DRIVER, Driver.EASON)));
                driver.add("Set Simple Driver", action("Set Simple Driver",
                        () -> Blackboard.put(Blackboard.Caption.DRIVER, Driver.SIMPLE)));
            });
            scope.folder("Pose", pose -> {
                pose.add("Set Red Default Start Pose", action("Set Red Default Start Pose",
                        () -> Blackboard.put(Blackboard.Caption.POSE, Constants.RED_DEFAULT_START_POSE.copy())));
                pose.add("Set Blue Default Start Pose", action("Set Blue Default Start Pose",
                        () -> Blackboard.put(Blackboard.Caption.POSE, Constants.BLUE_DEFAULT_START_POSE.copy())));
                pose.add("Set Zero Pose", action("Set Zero Pose",
                        () -> Blackboard.put(Blackboard.Caption.POSE, new Pose())));
            });
            scope.folder("Miscellaneous", miscellaneous ->
                    miscellaneous.add("Reset All Blackboard",
                            action("Reset All Blackboard", Blackboard::resetAll)));
        });
    }

    private static Supplier<OpMode> action(String name, Runnable action) {
        return () -> new BlackboardActionOpMode(name, action);
    }

    private static final class BlackboardActionOpMode extends OpMode {
        private final String actionName;
        private final Runnable action;

        private BlackboardActionOpMode(String actionName, Runnable action) {
            this.actionName = actionName;
            this.action = action;
        }

        @Override
        public void init() {
            action.run();
        }

        @Override
        public void init_loop() {
            telemetry.clear();
            telemetry.addLine("Applied: " + actionName);
            telemetry.addLine("Press Start to finish.");
            telemetry.update();
        }

        @Override
        public void loop() {
            requestOpModeStop();
        }
    }
}
