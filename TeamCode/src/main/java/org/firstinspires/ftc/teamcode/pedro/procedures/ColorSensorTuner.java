package org.firstinspires.ftc.teamcode.pedro.procedures;


import com.acmerobotics.dashboard.FtcDashboard;
import com.pedropathing.controllers.Controller;
import com.pedropathing.tuning.autotune.Inputs;
import com.pedropathing.tuning.autotune.Procedure;
import com.pedropathing.tuning.autotune.TuningOpMode;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.util.math.MathUtils;

public class ColorSensorTuner extends Procedure {
    public ColorSensorTuner() {
        super("Color Sensor Tuner", "A tuner for tuning a color sensor's gain values, among other things");
    }

    @Override
    public void run() throws InterruptedException {
        Inputs nameInput = inputs("Color Sensor HWMap name", "Enter the configured name of your color sensor");
        Inputs.Field<String> name = nameInput.s("HardwareMap Name").withDefault("color_sensor");
        awaitInputs(nameInput);

        double gain = runOpMode(new GainTuner(name.get()));
        result("name", name.get());
        result("gain", gain);

        code(Language.JAVA, "RevColorSensorV3 colorSensor = hardwareMap.get(RevColorSensorV3.class, \"" + name.get() + "\"));\n" +
                "colorSensor.setGain(" + gain + ");");
    }
}

class GainTuner extends TuningOpMode<Double> {
    String name;
    // wait i just realized ts runs like node page so how will gp access it?
    public GainTuner(String colorSensorName) {
        super("Color Sensor Gain Tuner", "Auto-tunes the gain of the specified Rev Color Sensor V3, " + colorSensorName, true);
        name = colorSensorName;
    }

    @Override
    protected Double runTuningOpMode() throws InterruptedException {
        RevColorSensorV3 colorSensorV3 = hardwareMap.get(RevColorSensorV3.class, name);
        Controller gainTuner = Controller.proportional(2);
        telemetry = FtcDashboard.getInstance().getTelemetry();
        telemetry.clearAll();
        double gain = 1;
        waitForStart();
        while (!isStopRequested()) {
            colorSensorV3.setGain((float) gain);
            NormalizedRGBA colors = colorSensorV3.getNormalizedColors();
            if (colors.alpha <= 1e-6) {
                telemetry.addLine("Alpha is too small; cannot normalize.");
                telemetry.update();
                continue;
            }
            telemetry.addData("Gain", gain);
            telemetry.addData("Color Alpha value", colors.alpha);
            telemetry.addData("Color Red value", colors.red);
            telemetry.addData("Color Green value", colors.green);
            telemetry.addData("Color Blue value", colors.blue);
            telemetry.addLine();
            telemetry.addData("Normalized Red", colors.red/colors.alpha);
            telemetry.addData("Normalized Green", colors.green/colors.alpha);
            telemetry.addData("Normalized Blue", colors.blue/colors.alpha);

            telemetry.addData("Distance (IN)", colorSensorV3.getDistance(DistanceUnit.INCH));
            telemetry.update();



            double max = MathUtils.maximum(colors.red/colors.alpha, colors.green/colors.alpha, colors.blue/colors.alpha);

            if (Math.abs(max - 1) <= 0.01) {
                telemetry.addData(" ====== RESULT ======", "\n FINISHED TUNING");
               break;
            }

            gain += gainTuner.calculate(1, 1-max); // target, error
        }
        return gain;
    }
}
