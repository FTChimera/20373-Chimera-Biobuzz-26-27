package org.firstinspires.ftc.teamcode.configs;

import com.pedropathing.geometry.Pose;
import com.pedropathing.telemetry.Selector;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.library.AllianceColor;
import org.firstinspires.ftc.teamcode.library.hardware.gamepad.Button;
import org.firstinspires.ftc.teamcode.library.hardware.gamepad.GamepadEx;
import org.firstinspires.ftc.teamcode.library.robot.Driver;

@TeleOp(name = "Configuration", group = "Configuration")
public class ConfigOpMode extends OpMode {
    private final Selector<Runnable> selector;
    private GamepadEx controller;
    private String lastAction = "";

    public ConfigOpMode() {
        selector = Selector.create("Robot Configuration", scope -> {
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
        }, new String[] {
                "D-pad up/down: move cursor",
                "Right bumper: open folder or apply selection",
                "Left bumper: go back"
        });
    }

    private Runnable action(String name, Runnable action) {
        return () -> {
            action.run();
            lastAction = "Applied: " + name;
        };
    }

    @Override
    public void init() {
        controller = new GamepadEx(gamepad1);
    }

    @Override
    public void init_loop() {
        controller.update();
        updateControls();

        telemetry.clear();
        for (String line : selector.getLines()) {
            telemetry.addLine(line);
        }
        if (!lastAction.isEmpty()) {
            telemetry.addLine();
            telemetry.addLine(lastAction);
        }
        telemetry.update();
    }

    private void updateControls() {
        if (controller.wasJustPressed(Button.DPAD_UP)) {
            selector.decrementSelected();
        } else if (controller.wasJustPressed(Button.DPAD_DOWN)) {
            selector.incrementSelected();
        } else if (controller.wasJustPressed(Button.RIGHT_BUMPER)) {
            selector.select();
        } else if (controller.wasJustPressed(Button.LEFT_BUMPER)) {
            selector.goBack();
        }
    }

    @Override
    public void loop() {
        requestOpModeStop();
    }
}
