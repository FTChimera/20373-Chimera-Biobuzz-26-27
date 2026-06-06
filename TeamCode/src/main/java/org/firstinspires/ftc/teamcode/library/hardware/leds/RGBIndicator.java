package org.firstinspires.ftc.teamcode.library.hardware.leds;

import org.firstinspires.ftc.teamcode.library.hardware.webcam.Limelight;
import org.firstinspires.ftc.teamcode.library.robot.RobotConstants;
import org.firstinspires.ftc.teamcode.mechanisms.Mechanism;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
@SuppressWarnings("SpellCheckingInspection")
/*
 * GOBILDA PRODUCT INSIGHT: https://cdn11.bigcommerce.com/s-x56mtydx1w/images/stencil/original/products/2275/15126/3118-0808-0002-Product-Insight-4__88285.1757516465.png?c=1
 */
public class RGBIndicator extends Mechanism {
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
        WHITE(WHITE_PWM);
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
    private final boolean isBeingUsed;
    public RGBIndicator(HardwareMap hardwareMap, RobotConstants bot) {
        super(hardwareMap, bot);
        this.rgb = hardwareMap.get(Servo.class, "rgb");
        rgb.setPosition(BLACK_PWM);
        isBeingUsed=bot.Mechanisms.contains(RGBIndicator.class);
    }
    public void setCustomPWM(double val) {
        if (!isBeingUsed) return;
        rgb.setPosition(val);
    }
    public void setColor(Color col) {
        if (!isBeingUsed) return;
        rgb.setPosition(col.getPWM());
    }
    public void updateUsingLL(Limelight ll) {
        if (!isBeingUsed) return;
        if (ll.isDisconnected) {
            this.setColor(RGBIndicator.Color.BLACK); // Limelight not looking at target
        }
        else if (ll.getRGBScore() < 0.9) {
            // GREEN
            this.setColor(RGBIndicator.Color.GREEN);
        } else if (ll.getRGBScore() < 5) {
            // YELLOW
            this.setColor(RGBIndicator.Color.YELLOW);
        } else if (ll.getRGBScore() < 12.5) {
            // ORANGE
            this.setColor(RGBIndicator.Color.ORANGE);
        } else {
            // OFF
            this.setColor(RGBIndicator.Color.BLACK);
        }
    }
}