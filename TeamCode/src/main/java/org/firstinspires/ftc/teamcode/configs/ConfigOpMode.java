package org.firstinspires.ftc.teamcode.configs;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.library.AllianceColor;
import org.firstinspires.ftc.teamcode.library.robot.Driver;

import java.util.HashMap;
import java.util.List;
import java.util.Map;


public class ConfigOpMode extends OpMode {
//replace map with a tuple
    private static class ConfigMap extends HashMap<String, List<Map<String, Runnable>>> {
        public ConfigMap(String str, List<Map<String, Runnable>> list) {
            super(Map.of(str, list));
        }
    }
    @FunctionalInterface
    private interface BlackboardPut {
        void put(Blackboard.Caption cap, Object object);
    }
    @FunctionalInterface
    private interface BlackboardGet {
        Object get(Blackboard.Caption cap);
    }
    BlackboardPut storer = ((cap, object) -> Blackboard.put(new Blackboard.Item(cap, object)));
    BlackboardGet getter = cap -> Blackboard.get(cap).getValue();
    List<ConfigMap> configuration;

    int selectedFolder = 0;
    int selectedRunnable = 0;
    boolean runnableSelection = false; // false for folder topic selection, turns true when it proceeds to subtopic

    public void loop() {
        requestOpModeStop();
    }

    @Override
    public void init() {

        ConfigMap AllianceColorPrefs = new ConfigMap("Alliance Color Prefs", List.of(
                Map.of("Set Alliance Color RED", () -> storer.put(Blackboard.Caption.ALLIANCE, AllianceColor.RED)),
                Map.of("Set Alliance Color BLUE", () -> storer.put(Blackboard.Caption.ALLIANCE, AllianceColor.BLUE))
        ));

        ConfigMap DriverPrefs = new ConfigMap("Driver Prefs", List.of(
                Map.of("Set Atharva to be Driver", () -> storer.put(Blackboard.Caption.DRIVER, Driver.ATHARVA)),
                Map.of("Set Shlok to be Driver", () -> storer.put(Blackboard.Caption.DRIVER, Driver.SHLOK)),
                Map.of("Set Eason to be Driver", () -> storer.put(Blackboard.Caption.DRIVER, Driver.EASON)),
                Map.of("Set Simple Driver", () -> storer.put(Blackboard.Caption.DRIVER, Driver.SIMPLE))
        ));

        ConfigMap PosePrefs = new ConfigMap("Pose Prefs", List.of(
                Map.of("Set Red Default Start Pose", () -> storer.put(Blackboard.Caption.POSE, Constants.RED_DEFAULT_START_POSE.copy())),
                Map.of("Set Blue Default Start Pose", () -> storer.put(Blackboard.Caption.POSE, Constants.BLUE_DEFAULT_START_POSE.copy())),
                Map.of("Set Zero Pose", () -> storer.put(Blackboard.Caption.POSE, new Pose().copy()))
        ));

        ConfigMap MiscPrefs = new ConfigMap("Miscellaneous", List.of(
                Map.of("Reset All Blackboard", Blackboard::resetAll)
        ));

        configuration = List.of(AllianceColorPrefs, DriverPrefs, PosePrefs, MiscPrefs);
    }

    public void init_loop() {
        // Controls
        displayControls(telemetry);
        if (!runnableSelection) {
            for (int i = 0; i < configuration.size(); i++) {
                telemetry.addLine(i==selectedFolder?"> " : "" + configuration.get(i).keySet().iterator().next());
            }
        } else {
            ConfigMap currentMap = configuration.get(selectedFolder);
            for (int i = 0; i < configuration.size(); i++) {
                telemetry.addLine(i==selectedRunnable?"> " : "" + currentMap.entrySet().iterator().next().getKey());
            }
        }
        
        updateControls(gamepad1);

        telemetry.update();
    }

    public void displayControls(Telemetry telemetry) {
        telemetry.clear();
        telemetry.addLine("dpad to move around");
        telemetry.addLine();
    }

    public void updateControls(Gamepad gamepad) {
        if ((gamepad.dpadLeftWasPressed && runnableSelection) || (gamepad.dpadRightWasPressed && !runnableSelection)) {
            runnableSelection = !runnableSelection;
        }

        if (gamepad.dpadRightWasPressed && runnableSelection) {
            configuration.get(selectedFolder).get(selectedRunnable).entrySet().iterator().next().getValue().run();
            telemetry.addLine("Executed: " + configuration.get(selectedFolder).get(selectedRunnable).entrySet().iterator().next().getKey());
        }

        if (runnableSelection) {
            int mv = gamepad.dpadUpWasPressed ? 1 : gamepad.dpadDownWasPressed ? -1 : 0;
            selectedRunnable = (selectedRunnable + mv + configuration.get(selectedFolder).size()) % configuration.get(selectedFolder).size();
        } else {
            int mv = gamepad.dpadUpWasPressed ? 1 : gamepad.dpadDownWasPressed ? -1 : 0;
            selectedFolder = (selectedFolder + mv + configuration.size()) % configuration.size();
        }
    }
}
