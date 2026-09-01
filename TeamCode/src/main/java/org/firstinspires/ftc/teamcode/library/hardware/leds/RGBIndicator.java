package org.firstinspires.ftc.teamcode.library.hardware.leds;

import org.firstinspires.ftc.teamcode.library.game.State;
import org.firstinspires.ftc.teamcode.library.game.RobotMode;
import org.firstinspires.ftc.teamcode.library.robot.RobotConstants;
import org.firstinspires.ftc.teamcode.mechanisms.Mechanism;

//import com.bylazar.lights.PanelsLights;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@SuppressWarnings("SpellCheckingInspection")
/*
 * GOBILDA PRODUCT INSIGHT: https://cdn11.bigcommerce.com/s-x56mtydx1w/images/stencil/original/products/2275/15126/3118-0808-0002-Product-Insight-4__88285.1757516465.png?c=1
 */
public class RGBIndicator extends Mechanism {
    /**
     * Returns an indication of the manufacturer of this device.
     *
     * @return the device's manufacturer
     */

    public static class RGBIndicator_PARAMS implements Mechanism.Params<RGBIndicator_PARAMS> {
        public enum Type {ONLY_HARDWARE, ONLY_VIRTUAL, BOTH, NAN; public boolean panels() {return this == ONLY_VIRTUAL || this == BOTH;} public boolean hardware() {return this == ONLY_HARDWARE || this == BOTH;}}
        public boolean ISBEINGUSED = true;
        public Type type = Type.ONLY_HARDWARE; // for now
        public String HARDWAREMAP_NAME = "rgb";
        public String PANELS_NAME = "RGB INDICATOR";
        public RGBIndicator_PARAMS() {}
        private RGBIndicator_PARAMS(RGBIndicator_PARAMS other) {
            ISBEINGUSED = other.ISBEINGUSED;
            type = other.type;
            HARDWAREMAP_NAME = other.HARDWAREMAP_NAME;
            PANELS_NAME = other.PANELS_NAME;
        }
        public RGBIndicator_PARAMS isBeingUsed(boolean isBeingUsed) {
            this.ISBEINGUSED = isBeingUsed;
            return this;
        }
        public RGBIndicator_PARAMS type(Type indicatorType) {
            this.type = indicatorType;
            return this;
        }
        public RGBIndicator_PARAMS hardwareMapName(String hardwareMapName) {
            this.HARDWAREMAP_NAME = hardwareMapName;
            return this;
        }
        public RGBIndicator_PARAMS panelsName(String panelsName) {
            this.PANELS_NAME = panelsName;
            return this;
        }
        @Override
        public RGBIndicator_PARAMS updateFromBot(RobotConstants bot) {
            if (!ISBEINGUSED) return this;
            ISBEINGUSED = bot.Mechanisms.contains(RGBIndicator.class);
            return this;
        }
        @Override
        public RGBIndicator_PARAMS copy() {
            return new RGBIndicator_PARAMS(this);
        }
    }
    public enum Color {
        BLACK(BLACK_PWM),
        RED(RED_PWM),
        ORANGE(ORANGE_PWM),
        GOLD(GOLD_PWM),
        YELLOW(YELLOW_PWM),
        SAGE(SAGE_PWM),
        GREEN(GREEN_PWM),
        AZURE(AZURE_PWM),
        BLUE(BLUE_PWM),
        INDIGO(INDIGO_PWM),
        VIOLET(VIOLET_PWM),
        WHITE(WHITE_PWM),
        UNKNOWN(BLACK_PWM);
        private final double pwmValue;
        Color(double pwmValue) {
            this.pwmValue = pwmValue;
        }
        public double getPWM() {
            return pwmValue;
        }
    }
    /*
    Black – 0
    Red – 0.279
    Orange – 0.333
    Gold – 0.357
    Yellow – 0.388
    Sage – 0.444
    Green – 0.500
    Azure – 0.555
    Blue – 0.611
    Indigo – 0.666
    Violet – 0.722
    White – 1
    */
    public static double BLACK_PWM = 0.0;
    public static double RED_PWM = 0.279;
    public static double ORANGE_PWM = 0.333;
    public static double GOLD_PWM = 0.357;
    public static double YELLOW_PWM = 0.388;
    public static double SAGE_PWM = 0.444;
    public static double GREEN_PWM = 0.500;
    public static double AZURE_PWM = 0.555;
    public static double BLUE_PWM = 0.611;
    public static double INDIGO_PWM = 0.666;
    public static double VIOLET_PWM = 0.722;
    public static double WHITE_PWM = 1.0;

    private Servo rgb;
    public RGBIndicator_PARAMS params;
    private final boolean isBeingUsed;
//    com.bylazar.lights.RGBIndicator panelsRGB;
    public RGBIndicator(HardwareMap hardwareMap, RobotConstants bot) {
        super(hardwareMap, bot);
        this.params = resolveParams(RGBIndicator.class, new RGBIndicator_PARAMS());
        isBeingUsed = params.ISBEINGUSED;
        if (!isBeingUsed) return;
        this.rgb = hardwareMap.get(Servo.class, params.HARDWAREMAP_NAME);
        rgb.setPosition(BLACK_PWM);
        if (params.type.panels()) {
//            panelsRGB = new com.bylazar.lights.RGBIndicator(params.PANELS_NAME);
//            PanelsLights.INSTANCE.getLights().initLights(panelsRGB);
        }
    }

    @Override
    public void stop() {
        setColor(Color.BLACK);
    }

    public void setCustomPWM(double val) {
        if (!isBeingUsed) return;
        internalSetPwm(val);
    }
    public void setColor(Color col) {
        if (!isBeingUsed) return;
        internalSetPwm(col.getPWM());
    }
    public double getPWM() {
        if (!isBeingUsed) return 0.0;
        if (params.type.hardware()) return rgb.getPosition();
        //if (params.type.panels()) return panelsRGB.getValue();
        return 0.0;
    }

    public Color getColor() {
        if (!isBeingUsed) return Color.UNKNOWN;
        double pwm = getPWM();
        for (Color color : Color.values()) {
            if (Math.abs(color.getPWM() - pwm) < 0.01) {
                return color;
            }
        }
        return Color.UNKNOWN;
    }

    private void internalSetPwm(double pwm) {
        if (params.type.hardware()) {
                rgb.setPosition(pwm);
        }
        if (params.type.panels()) {
//            panelsRGB.update(pwm);
//            PanelsLights.INSTANCE.getLights().update();
        }
    }

    @Override
    public void periodic(State state, RobotMode robotMode) {}

}