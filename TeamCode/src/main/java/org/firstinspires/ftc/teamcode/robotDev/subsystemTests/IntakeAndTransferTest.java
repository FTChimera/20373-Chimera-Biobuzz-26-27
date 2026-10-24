package org.firstinspires.ftc.teamcode.robotDev.subsystemTests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.ValueProvider;
import com.acmerobotics.dashboard.config.VariableProvider;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Utility;
import com.qualcomm.robotcore.hardware.DcMotor;


@Utility
public class IntakeAndTransferTest extends OpMode {
    DcMotor transferMotor;
    ValueProvider<Double> powerProvider = new VariableProvider<>(0.0);
    ValueProvider<Double> transferProvider = new VariableProvider<>(0.0);
    DcMotor intakeMotor;
    public void init() {
        transferMotor = hardwareMap.dcMotor.get("transfer");
        intakeMotor = hardwareMap.dcMotor.get("intake");
        FtcDashboard.getInstance().addConfigVariable("test", "Intake Power", powerProvider);
        FtcDashboard.getInstance().addConfigVariable("test", "Transfer Power", transferProvider);
    }
    public void loop() {
        transferMotor.setPower(transferProvider.get());
        intakeMotor.setPower(powerProvider.get());
        FtcDashboard.getInstance().getTelemetry().addData("Transfer Power", transferMotor.getPower());
        FtcDashboard.getInstance().getTelemetry().addData("Intake Power", intakeMotor.getPower());
        FtcDashboard.getInstance().getTelemetry().update();
    }
}
