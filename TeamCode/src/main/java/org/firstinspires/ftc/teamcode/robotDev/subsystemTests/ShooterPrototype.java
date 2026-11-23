package org.firstinspires.ftc.teamcode.robotDev.subsystemTests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.ValueProvider;
import com.acmerobotics.dashboard.config.VariableProvider;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Utility;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import com.rmsftc.utils.config.vars.Var;
import org.firstinspires.ftc.teamcode.util.dashboard.DashboardVars;

@Utility()
public class ShooterPrototype extends LinearOpMode {
    // Velocity in ticks/sec; 3000 ceiling is above the 6000 RPM motor's
    // theoretical max (~2800 ticks/s at 28 ticks/rev), 0 floor = no reverse.
    Var<Integer> var = DashboardVars.intVar("Shooter Speed", 700, 0, 3000);
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx prototype = hardwareMap.get(DcMotorEx.class, "mainMotor");
        prototype.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        var.section.category = "test";
        DashboardVars.upload(var);
        telemetry = FtcDashboard.getInstance().getTelemetry();
        while (opModeInInit()) {
            telemetry.addData("Initialized. default speed = ", var.get());
            telemetry.addData("Direction", prototype.getDirection());
            telemetry.update();
        }
        waitForStart();
        while (opModeIsActive()) {

            prototype.setVelocity(var.get());
            double getv = prototype.getVelocity();


            telemetry.addData("prototype get velocity", getv);
            telemetry.addData("error", var.get() - getv);
            telemetry.update();
        }
        prototype.setVelocity(0);
    }
}

