package org.firstinspires.ftc.teamcode.util.hardware.sensors;



import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.teamcode.util.Color;
import org.firstinspires.ftc.teamcode.util.game.BallCol;

import java.util.InvalidPropertiesFormatException;

public class ColorSensor {
    RevColorSensorV3 revColorSensor;
    public static double COST_THRESHOLD_DEFAULT = 0.06;
    // TODO: Known bugs
    // RED nectar only detected up close. Turns into unknown then pollen yellow when move farther away.
    Result cachedResult = null;
    public Color defaultColor;
    public double costThreshold;

    // default color is the wall color in front of color sensor
    public ColorSensor(HardwareMap hardwareMap, String name, Color defaultColor, float gain, double cost_threshold) {
        revColorSensor = hardwareMap.get(RevColorSensorV3.class, name);
        revColorSensor.setGain(gain);
        this.defaultColor = defaultColor;
        this.costThreshold = cost_threshold;
    }

    public void update() {
        NormalizedRGBA colors = revColorSensor.getNormalizedColors();
//        float normRed, normGreen, normBlue;
//        normRed = colors.red/colors.alpha;
//        normGreen = colors.green/colors.alpha;
//        normBlue = colors.blue/colors.alpha;

        cachedResult = new Result(colors);
    }

    public Result getLatestResult() {
        return cachedResult;
    }

    public BallCol getBallCol() throws InvalidPropertiesFormatException {
        if (this.defaultColor==Color.YELLOW || this.defaultColor == Color.RED || this.defaultColor == Color.BLUE)
            throw new InvalidPropertiesFormatException("HOW ARE WE SUPPOSED TO FIND THE BALL COLOR IF THE WALL IS THE SAME COLOR");
        return BallCol.detect(cachedResult.colors, costThreshold);
    }


    public static class Result {
        public float red;
        public float green;
        public float blue;
        public float alpha;
        public int detectedColor;
        public float sum;

        public NormalizedRGBA colors;

        public Result(NormalizedRGBA colors) {
            this.red = colors.red;
            this.green = colors.green;
            this.blue = colors.blue;
            this.alpha = colors.alpha;
            this.sum = red + green + blue;
            this.detectedColor = colors.toColor();
            this.colors = colors;
        }

    }
}
