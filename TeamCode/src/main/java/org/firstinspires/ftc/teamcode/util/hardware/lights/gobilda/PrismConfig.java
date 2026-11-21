package org.firstinspires.ftc.teamcode.util.hardware.lights.gobilda;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Utility;

import org.firstinspires.ftc.teamcode.util.Color;
import org.firstinspires.ftc.teamcode.util.Direction;

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

    // Artboard map (keep in sync with Lights.BaseStatus):
    //   0 team color (idle) · 1 rainbow party · 2 solid green (ready)
    //   3 team-blue pulse · 4 fast red blink (last 7 s) · 5 rainbow snakes
    //   6 team-color snakes (one-minute mark) · 7 sine wave
    public void loadAnimationArtboard(GoBildaPrismDriver.Artboard artboard) {
        switch (artboard) {
            case ARTBOARD_0:
                prism.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_0, PrismAnimations.sineWave().primaryColor(Color.TEAM_COLOR_BLUE).secondaryColor(Color.TEAM_COLOR_BLUE.dimColor(4.0F)).direction(Direction.FORWARD).speed(0.4F).period(1000).build());
                break;
            case ARTBOARD_1:
                prism.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_0, PrismAnimations.rainbow().brightness(30).build());
                break;
            case ARTBOARD_2:
                prism.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_0, PrismAnimations.solid().primaryColor(Color.GREEN).build());
                break;
            case ARTBOARD_3:
                prism.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_0, PrismAnimations.pulse().primaryColor(Color.TEAM_COLOR_BLUE).secondaryColor(Color.TEAM_COLOR_BLUE.dimColor()).build());
                break;
            case ARTBOARD_4:
                prism.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_0, PrismAnimations.blink().primaryColor(Color.RED).secondaryColor(Color.TRANSPARENT).period(400).primaryColorPeriod(200).build());
                break;
            case ARTBOARD_5:
                prism.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_0, new PrismAnimations.RainbowSnakes());
                break;
            case ARTBOARD_6:
                prism.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_0, PrismAnimations.snakes().colors(Color.TEAM_COLOR_BLUE, Color.TEAM_COLOR_WHITE, Color.TEAM_COLOR_BLUE, Color.TEAM_COLOR_WHITE).build());
                break;
            case ARTBOARD_7:
                prism.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_0, new PrismAnimations.SineWave());
                break;
        }

        sleep(200);
    }

}
