package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Utility;

import org.firstinspires.ftc.teamcode.mechanisms.Lights;
import org.firstinspires.ftc.teamcode.util.AllianceColor;

/** When the robot is on a display stand*/
@Utility
public class DisplayStand extends LinearOpMode {
    // TODO: Implement
    @Override
    public void runOpMode() {

        waitForStart();
        Lights lights = new Lights(hardwareMap, AllianceColor.RED);
        lights.clearAll();
        lights.rainbowParty();
    }
}
