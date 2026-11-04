package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.AllianceColor;

public class StubVision extends Vision {
    public StubVision(HardwareMap hardwareMap, AllianceColor allianceColor) {
        super(hardwareMap, allianceColor);
    }

    @Override
    public void start() {}

    @Override
    public void stop() {}

    @Override
    public LLResult getLatestResult() { return null; }

    @Override
    public void update(double robotHeadingRadians) {}

    @Override
    public boolean hasReliableLocalization() {return false;}


}
