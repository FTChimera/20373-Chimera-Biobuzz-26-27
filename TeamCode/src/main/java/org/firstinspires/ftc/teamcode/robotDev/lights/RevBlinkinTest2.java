package org.firstinspires.ftc.teamcode.robotDev.lights;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Utility;


@Utility
public class RevBlinkinTest2 extends LinearOpMode {

    /**
     * Override this method and place your code here.
     * <p>
     * Please do not catch {@link InterruptedException}s that are thrown in your OpMode
     * unless you are doing it to perform some brief cleanup, in which case you must exit
     * immediately afterward. Once the OpMode has been told to stop, your ability to
     * control hardware will be limited.
     *
     * @throws InterruptedException When the OpMode is stopped while calling a method
     *                              that can throw {@link InterruptedException}
     */
    @Override
    public void runOpMode() throws InterruptedException {
        RevBlinkinLedDriver s = hardwareMap.get(RevBlinkinLedDriver.class, "ledStrip");
        s.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
        waitForStart();
    }
}
