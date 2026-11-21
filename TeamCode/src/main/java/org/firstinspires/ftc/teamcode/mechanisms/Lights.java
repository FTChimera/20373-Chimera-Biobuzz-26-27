package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.AllianceColor;
import org.firstinspires.ftc.teamcode.util.Color;
import org.firstinspires.ftc.teamcode.util.Direction;
import org.firstinspires.ftc.teamcode.util.game.BallCol;
import org.firstinspires.ftc.teamcode.util.hardware.lights.gobilda.GoBildaPrismDriver;
import org.firstinspires.ftc.teamcode.util.hardware.lights.gobilda.PrismAnimations;


/**
 * Prism LED control.
 *
 * Two display layers:
 *  - Base status: a full-strip scene loaded from a Prism artboard (programmed
 *    once by the PrismConfig utility OpMode). Artboards are only reloaded
 *    when the status actually changes, not every loop.
 *  - Event feedback: short one-shot animations inserted on the top layer
 *    (LAYER_9) when the driver does something in teleop (re-zero, intake
 *    toggle, deployer, eject, shoot, panic, park). Black is transparent on
 *    the Prism, so the base status shows through between flashes. Each event
 *    animation expires after a fixed time and is replaced with a transparent
 *    solid. A base-status change (artboard load) also ends any event.
 */
public class Lights {
    public GoBildaPrismDriver prism;
    public AllianceColor allianceColor;
    private final LightStrips lightStrips = new LightStrips(
            LightSize.TWELVE_LED_STRIP_200MM,
            LightSize.SIX_LED_STRIP_104MM,
            LightSize.SIX_LED_STRIP_104MM
    );

    // Base status scenes, one artboard each. Keep in sync with PrismConfig,
    // which programs these artboards onto the Prism.
    private enum BaseStatus {
        OFF,
        TEAM_COLOR,   // ARTBOARD_0: team-blue sine wave
        PARTY,        // ARTBOARD_1: rainbow
        READY_GREEN,  // ARTBOARD_2: solid green (shooter up to speed)
        FLASH_RED,    // ARTBOARD_4: fast red blink (last 7 seconds)
        ONE_MINUTE    // ARTBOARD_6: team-color snakes (58-61 s remaining)
    }

    private BaseStatus currentBase = null;

    // Transient driver-action feedback lives on the top layer so it draws
    // over whatever base status is showing.
    private static final GoBildaPrismDriver.LayerHeight EVENT_LAYER =
            GoBildaPrismDriver.LayerHeight.LAYER_9;
    private boolean eventActive = false;
    private long eventEndTimeMs = 0;

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
        try {
            prism = hardwareMap.get(GoBildaPrismDriver.class, "prism");
        } catch (RuntimeException ignored) {
            if (false) {}//this instanceof StubLights
            else {
                throw new IllegalStateException("Prism not found in config");
            }
        }
        prism.clearAllAnimations();
        this.allianceColor = allianceColor;
        //displayAllianceColor();
        displayTeamColor();
    }

    public void start() {
        displayTeamColor();
    }

    public void update() {
        if (eventActive && System.currentTimeMillis() >= eventEndTimeMs) {
            clearEventAnimation();
        }
        prism.updateAllAnimations();
    }

    // ---- Base status (artboards) ----

    private void showBase(BaseStatus status) {
        if (status == currentBase) return;
        currentBase = status;
        // Loading an artboard replaces the device's animation slots, so any
        // event animation on the top layer is gone; forget it locally too.
        eventActive = false;
        switch (status) {
            case OFF:
                prism.clearAllAnimations();
                break;
            case TEAM_COLOR:
                prism.loadAnimationsFromArtboard(GoBildaPrismDriver.Artboard.ARTBOARD_0);
                break;
            case PARTY:
                prism.loadAnimationsFromArtboard(GoBildaPrismDriver.Artboard.ARTBOARD_1);
                break;
            case READY_GREEN:
                prism.loadAnimationsFromArtboard(GoBildaPrismDriver.Artboard.ARTBOARD_2);
                break;
            case FLASH_RED:
                prism.loadAnimationsFromArtboard(GoBildaPrismDriver.Artboard.ARTBOARD_4);
                break;
            case ONE_MINUTE:
                prism.loadAnimationsFromArtboard(GoBildaPrismDriver.Artboard.ARTBOARD_6);
                break;
        }
    }

    public void displayTeamColor() {
        showBase(BaseStatus.TEAM_COLOR);
    }

    public void rainbowParty() {
        showBase(BaseStatus.PARTY);
    }

    public void displayGreen() {
        showBase(BaseStatus.READY_GREEN);
    }

    public void flashRed() {
        showBase(BaseStatus.FLASH_RED);
    }

    public void oneMinThing() {
        showBase(BaseStatus.ONE_MINUTE);
    }

    public void off() {
        showBase(BaseStatus.OFF);
    }

    public void clearAll() {
        currentBase = BaseStatus.OFF;
        eventActive = false;
        prism.clearAllAnimations();
    }

    public void stop() {
        clearAll();
        rainbowParty();
    }

    // ---- Event feedback (top layer, one-shot) ----

    private void playEventAnimation(PrismAnimations.AnimationBase eventAnimation, long durationMs) {
        prism.insertAndUpdateAnimation(EVENT_LAYER, eventAnimation);
        eventActive = true;
        eventEndTimeMs = System.currentTimeMillis() + durationMs;
    }

    private void clearEventAnimation() {
        prism.insertAndUpdateAnimation(EVENT_LAYER, PrismAnimations
                .solid()
                .primaryColor(Color.TRANSPARENT)
                .brightness(0)
                .indexes(0, lightStrips.numLights() - 1)
                .build());
        eventActive = false;
    }

    /** Ends any running event animation right away. */
    public void stopEventAnimation() {
        if (eventActive) clearEventAnimation();
    }

    /** Corner re-zero applied: white double-blink. */
    public void onReZero() {
        playEventAnimation(PrismAnimations
                .blink()
                .primaryColor(Color.WHITE)
                .secondaryColor(Color.TRANSPARENT)
                .period(300)
                .primaryColorPeriod(150)
                .indexes(0, lightStrips.numLights() - 1)
                .build(), 600);
    }

    /** Intake toggled on: cyan pulse. */
    public void onIntakeOn() {
        playEventAnimation(PrismAnimations
                .pulse()
                .primaryColor(Color.CYAN)
                .secondaryColor(Color.TRANSPARENT)
                .period(600)
                .indexes(0, lightStrips.numLights() - 1)
                .build(), 600);
    }

    /** Intake toggled off: single orange blink. */
    public void onIntakeOff() {
        playEventAnimation(PrismAnimations
                .blink()
                .primaryColor(Color.ORANGE)
                .secondaryColor(Color.TRANSPARENT)
                .period(400)
                .primaryColorPeriod(200)
                .indexes(0, lightStrips.numLights() - 1)
                .build(), 400);
    }

    /** Deployer ramp toggled: cyan double-blink deployed, purple stowed. */
    public void onDeployerToggle(boolean deployed) {
        playEventAnimation(PrismAnimations
                .blink()
                .primaryColor(deployed ? Color.CYAN : Color.PURPLE)
                .secondaryColor(Color.TRANSPARENT)
                .period(300)
                .primaryColorPeriod(150)
                .indexes(0, lightStrips.numLights() - 1)
                .build(), 600);
    }

    /** Eject started: fast red strobe. */
    public void onEjectStart() {
        playEventAnimation(PrismAnimations
                .blink()
                .primaryColor(Color.RED)
                .secondaryColor(Color.TRANSPARENT)
                .period(200)
                .primaryColorPeriod(100)
                .indexes(0, lightStrips.numLights() - 1)
                .build(), 1000);
    }

    /** Shooter spin-up started (Y or keep-shooter-on): white sparkle. */
    public void onShootStart() {
        playEventAnimation(PrismAnimations
                .sparkle()
                .period(100)
                .indexes(0, lightStrips.numLights() - 1)
                .build(), 800);
    }

    /** Panic shoot: police lights. */
    public void onPanicStart() {
        playEventAnimation(PrismAnimations
                .policeLights()
                .period(400)
                .indexes(0, lightStrips.numLights() - 1)
                .build(), 1500);
    }

    /** Park drive started: team-blue sine wave. */
    public void onParkStart() {
        playEventAnimation(PrismAnimations
                .sineWave()
                .primaryColor(Color.TEAM_COLOR_BLUE)
                .secondaryColor(Color.TEAM_COLOR_BLUE.dimColor(4.0F))
                .direction(Direction.FORWARD)
                .speed(0.4F)
                .period(1000)
                .indexes(0, lightStrips.numLights() - 1)
                .build(), 1000);
    }

    // ---- Code-built displays (not artboard-based) ----

    public void displayTransferColor(BallCol ballCol, boolean readyToShoot) {
        Color color = Color.fromBallColor(ballCol);
        Color shootReady = readyToShoot? Color.GREEN : Color.ORANGE.dimColor();

        currentBase = null; // code-built scene; next status call reloads its artboard
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
        currentBase = null; // code-built scene; next status call reloads its artboard
        prism.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_0, animation); //TODO: need to tune this
    }

    public String getDebugString() {return "Gobilda prism indicator";} // TODO

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

//    public static class StubLights extends Lights {
//
//        public StubLights(HardwareMap hardwareMap, AllianceColor allianceColor) {
//            super(hardwareMap, allianceColor);
//        }
//
//        @Override
//        public void update() {
//            prism.clearAllAnimations();
//        }
//
//    }
}
