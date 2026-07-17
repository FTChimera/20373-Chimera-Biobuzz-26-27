package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.library.hardware.hardwareDevice.SlidesMotor;
import org.firstinspires.ftc.teamcode.library.robot.RobotConstants;

public class Slides1 extends Mechanism {
    public static class SLIDES1_PARAMS implements Mechanism.Params<SLIDES1_PARAMS> {
        public String HARDWAREMAP_NAME = "Slides";
        public boolean ISBEINGUSED = true;
        public SLIDES1_PARAMS() {}
        private SLIDES1_PARAMS(SLIDES1_PARAMS other) {
            HARDWAREMAP_NAME = other.HARDWAREMAP_NAME;
            ISBEINGUSED = other.ISBEINGUSED;
        }
        public SLIDES1_PARAMS hardwareMapName(String hardwareMapName) {
            this.HARDWAREMAP_NAME = hardwareMapName;
            return this;
        }
        public SLIDES1_PARAMS isBeingUsed(boolean isBeingUsed) {
            this.ISBEINGUSED = isBeingUsed;
            return this;
        }
        @Override
        public SLIDES1_PARAMS updateFromBot(RobotConstants bot) {
            ISBEINGUSED = bot.Mechanisms.contains(Slides1.class);
            return this;
        }
        @Override
        public SLIDES1_PARAMS copy() {
            return new SLIDES1_PARAMS(this);
        }
    }
    // single motor slide
    private SlidesMotor motor;
    SLIDES1_PARAMS params;
    public Slides1(HardwareMap map, RobotConstants bot) {
        super(map, bot);
        params = resolveParams(Slides1.class, new SLIDES1_PARAMS());
        if (!params.ISBEINGUSED) return;
        motor = map.get(SlidesMotor.class, params.HARDWAREMAP_NAME);
    }

    

}
