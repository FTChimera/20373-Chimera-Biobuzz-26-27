package org.firstinspires.ftc.teamcode.util.hardware.lights.gobilda;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Utility;

@Utility
public class PrismConfig extends LinearOpMode {
    GoBildaPrismDriver prism;

    @Override
    public void runOpMode() throws InterruptedException {
        prism = hardwareMap.get(GoBildaPrismDriver.class, "prism");

        prism.clearAllAnimations();
        waitForStart();
        for (int i = 0; i < 8; i++) {
            prism.clearAllAnimations();
            loadAnimationArtboard(GoBildaPrismDriver.Artboard.fromIndex(i));
            prism.saveCurrentAnimationsToArtboard(GoBildaPrismDriver.Artboard.fromIndex(i));
            sleep(500);
        }
    }

    public void loadAnimationArtboard(GoBildaPrismDriver.Artboard artboard) {
        switch (artboard) {
            case ARTBOARD_0:
                PrismAnimations.AnimationBase animation1 = new PrismAnimations.PoliceLights();
                prism.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_0, animation1);
                break;
            case ARTBOARD_1:
                prism.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_0, new PrismAnimations.Rainbow());
                break;
            case ARTBOARD_2:
                    prism.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_0, new PrismAnimations.Blink());
                break;
            case ARTBOARD_3:
                prism.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_0, new PrismAnimations.Blink());
                break;
            case ARTBOARD_4:
                prism.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_0, new PrismAnimations.Blink());
                break;
            case ARTBOARD_5:
                prism.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_0, new PrismAnimations.RainbowSnakes());
                break;
            case ARTBOARD_6:
                prism.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_0, new PrismAnimations.Snakes());
                break;
            case ARTBOARD_7:
                prism.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_0   , new PrismAnimations.SineWave());
                break;
        }

        sleep(200);
    }

}
