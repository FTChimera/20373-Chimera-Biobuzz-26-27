package org.firstinspires.ftc.teamcode.robotDev.subsystemTests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Utility;
import com.qualcomm.robotcore.hardware.DcMotor;


@Utility
public class TransferTester extends OpMode {
    DcMotor transferMotor;
    public void init() {
        transferMotor = hardwareMap.dcMotor.get("transfer");
    }
    public void loop() {
        transferMotor.setPower(-1);
        FtcDashboard.getInstance().getTelemetry().addData("Transfer Power", transferMotor.getPower());
    }
}
