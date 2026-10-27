package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Utility;

import org.firstinspires.ftc.teamcode.mechanisms.Lights;
import org.firstinspires.ftc.teamcode.util.AllianceColor;
import org.firstinspires.ftc.teamcode.util.hardware.lights.gobilda.GoBildaPrismDriver;

/** When the robot is on a display stand*/
@Utility
public class DisplayStand extends LinearOpMode {
    // TODO: Implement
    @Override
    public void runOpMode() {
        Lights lights = new Lights(hardwareMap, AllianceColor.RED);
        lights.clearAll();
        waitForStart();
        lights.prism.loadAnimationsFromArtboard(GoBildaPrismDriver.Artboard.ARTBOARD_1);
        sleep(1000);
    }
}
