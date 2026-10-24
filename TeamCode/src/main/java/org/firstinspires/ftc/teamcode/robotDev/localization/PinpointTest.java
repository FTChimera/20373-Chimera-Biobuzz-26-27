package org.firstinspires.ftc.teamcode.robotDev.localization;

import com.pedropathing.localization.Localizer;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Utility;

import org.firstinspires.ftc.teamcode.pedro.Constants;

@Utility(name = "Pinpoint Test", description = "Test the pinpoint localizer")
public class PinpointTest extends OpMode {
    Localizer pinpoint;
    @Override
    public void init() {
        Constants.PINPOINT_CONFIG.resetMode.set(PinpointLocalizer.ResetMode.RESET_AND_RECALIBRATE_IMU);
        pinpoint = new PinpointLocalizer(hardwareMap, Constants.PINPOINT_CONFIG);
        pinpoint.reset();
    }

    @Override
    public void loop() {
        pinpoint.update();
        pinpoint.debug().forEach(telemetry::addData);
        telemetry.update();
    }
}
