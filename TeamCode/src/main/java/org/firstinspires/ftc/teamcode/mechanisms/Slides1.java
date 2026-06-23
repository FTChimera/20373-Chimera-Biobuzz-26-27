package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.library.hardware.hardwareDevice.SlidesMotor;
import org.firstinspires.ftc.teamcode.library.robot.RobotConstants;

public class Slides1 extends Mechanism {
    public static class SLIDES1_PARAMS {
        public String HARDWAREMAP_NAME = "Slides";
        public boolean ISBEINGUSED = true;
        public void updateFromBot(RobotConstants bot) {
            ISBEINGUSED = bot.Mechanisms.contains(Slides1.class);
        }
    }
    // single motor slide
    private SlidesMotor motor;
    SLIDES1_PARAMS params;
    public Slides1(HardwareMap map, RobotConstants bot) {
        super(map, bot);
        params = new SLIDES1_PARAMS();
        params.updateFromBot(bot);
        if (!params.ISBEINGUSED) return;
        motor = map.get(SlidesMotor.class, params.HARDWAREMAP_NAME);
    }

    

}
