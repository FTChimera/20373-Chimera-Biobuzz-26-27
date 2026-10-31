package org.firstinspires.ftc.teamcode.util.hardware.lights.gobilda;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Utility;

@Utility
public class testSnakes extends OpMode {
    GoBildaPrismDriver prismDriver;
    /**
     * User-defined init method
     * <p>
     * This method will be called once, when the INIT button is pressed.
     */
    @Override
    public void init() {
        prismDriver = hardwareMap.get(GoBildaPrismDriver.class, "prism");
        prismDriver.clearAllAnimations();
    }

    @Override public void start() {
        prismDriver.loadAnimationsFromArtboard(GoBildaPrismDriver.Artboard.ARTBOARD_6);
    }
    /**
     * User-defined loop method
     * <p>
     * This method will be called repeatedly during the period between when
     * the play button is pressed and when the OpMode is stopped.
     */
    @Override
    public void loop() {

    }
}
