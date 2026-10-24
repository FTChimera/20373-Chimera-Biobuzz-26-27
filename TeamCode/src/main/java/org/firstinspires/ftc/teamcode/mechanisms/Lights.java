package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.AllianceColor;
import org.firstinspires.ftc.teamcode.util.Color;
import org.firstinspires.ftc.teamcode.util.Direction;
import org.firstinspires.ftc.teamcode.util.game.BallCol;
import org.firstinspires.ftc.teamcode.util.hardware.lights.gobilda.GoBildaPrismDriver;
import org.firstinspires.ftc.teamcode.util.hardware.lights.gobilda.PrismAnimations;


public class Lights {
    public GoBildaPrismDriver prism;
    public AllianceColor allianceColor;
    private final LightStrips lightStrips = new LightStrips(
            LightSize.SIX_LED_STRIP_104MM,
            LightSize.TWELVE_LED_STRIP_200MM,
            LightSize.SIX_LED_STRIP_104MM,
            LightSize.TWELVE_LED_STRIP_200MM,
            LightSize.TWELVE_LED_STRIP_200MM,
            LightSize.TWELVE_LED_STRIP_200MM
    );

    public PrismAnimations.Pulse.Builder ballColorAnimation = PrismAnimations
            .pulse()
            .period(500)
            .indexes(0, lightStrips.numLights()/2 - 1);
     public PrismAnimations.Blink.Builder shootReadyAnimation = PrismAnimations
            .blink()
            .period(500)
            .indexes(lightStrips.numLights()/2, lightStrips.numLights()-1);

     public PrismAnimations.SineWave teamColorAnimation = PrismAnimations
             .sineWave()
             .primaryColor(Color.TEAM_COLOR_BLUE)
             .secondaryColor(Color.TEAM_COLOR_BLUE.dimColor(4.0F))
             .direction(Direction.FORWARD)
             .speed(0.4F)
             .period(1000)
             .indexes(0, lightStrips.numLights()-1)
             .build();

    PrismAnimations.AnimationBase animation = PrismAnimations
            .blink()
            .primaryColor(Color.RED)
            .period(1200)
            .primaryColorPeriod(800)
            .indexes(0, lightStrips.numLights()-1)
            .build();

    public Lights(HardwareMap hardwareMap, AllianceColor allianceColor) {
        prism = hardwareMap.get(GoBildaPrismDriver.class, "prism");
        prism.clearAllAnimations();
        this.allianceColor = allianceColor;
        //displayAllianceColor();
        rainbowParty();
    }

    public void start() {

    }

    public void rainbowParty() {
        PrismAnimations.AnimationBase animation = prism
                .rainbow()
                .speed(0.2F)
                .startHue(0.0F)
                .stopHue(1.0F)
                .repeatAfter(3)
                .indexes(0, lightStrips.numLights()-1)
                .build();
        prism.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_0, animation); //TODO: need to tune this
    }

    public void update(BallCol transferCol, boolean displayTransferColor, boolean readyToShoot, boolean closeToEndOfMatch) {
        prism.updateAllAnimations();
    }

    public void displayTransferColor(BallCol ballCol, boolean readyToShoot) {
        Color color = Color.fromBallColor(ballCol);
        Color shootReady = readyToShoot? Color.GREEN : Color.ORANGE.dimColor();

        prism.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_0,
                ballColorAnimation.
                        primaryColor(color).
                        secondaryColor(color.dimColor(1.4)).
                        build());
        prism.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_0,
                shootReadyAnimation.
                        primaryColor(shootReady).
                        secondaryColor(shootReady.dimColor()).
                        build());
    }

    public void displayTeamColor() {
        prism.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_0, teamColorAnimation); //TODO: need to tune this
    }

    public void flashRed() {
        prism.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_9, animation); //TODO: need to tune this
    }

    public void displayAllianceColor() {
        Color primary = Color.fromAlliance(allianceColor);
        Color dimmed = Color.dimColor(primary, 4.0);
        PrismAnimations.AnimationBase animation = prism
                .sineWave()
                .direction(Direction.FORWARD)
                .primaryColor(primary)
                .secondaryColor(dimmed)
                .speed(0.4F)
                .period(1000)
                .indexes(0, lightStrips.numLights()-1)
                .build();
        prism.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_0, animation); //TODO: need to tune this
    }


    public void stop() {
        prism.clearAllAnimations();
        rainbowParty();
    }

    public String getDebugString() {return "Gobilda prism indicator";} // TODO

    public void clearAll() {
        prism.clearAllAnimations();
    }


    private enum LightSize {SIX_LED_STRIP_104MM(6), TWELVE_LED_STRIP_200MM(12); private final int numLEDs; LightSize(int numLEDs) { this.numLEDs = numLEDs; } public int getNumLEDs() { return numLEDs; } }
    private static class LightStrips {
        LightSize[] lightSizes;
        private final int numLights;
        LightStrips(LightSize... lightSizes) {
            this.lightSizes = lightSizes;
            this.numLights = getTotalNumLEDs();
        }
        private int getTotalNumLEDs() {
            int total = 0;
            for (LightSize size : lightSizes) {
                total += size.getNumLEDs();
            }
            return total;
        }
        int numLights() {
            return numLights;
        }
    }
}
