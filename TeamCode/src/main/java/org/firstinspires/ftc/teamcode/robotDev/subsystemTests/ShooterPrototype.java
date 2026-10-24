package org.firstinspires.ftc.teamcode.robotDev.subsystemTests;

import static org.firstinspires.ftc.teamcode.util.vars.kt.TypesKt.IntVar;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.ValueProvider;
import com.acmerobotics.dashboard.config.VariableProvider;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Utility;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.util.vars.Var;
import org.firstinspires.ftc.teamcode.util.vars.kt.TypesKt;

@Utility()
public class ShooterPrototype extends LinearOpMode {
    Var<Integer> var = IntVar("Shooter Speed", 700);
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx prototype = hardwareMap.get(DcMotorEx.class, "shooter");
        prototype.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        var.section.category = "test";
        var.uploadToFtcDashboard();
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
