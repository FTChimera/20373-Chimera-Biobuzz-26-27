package org.firstinspires.ftc.teamcode.learning.flyset;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class PIDFExample extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Control control = new Control();
        Sensor sensor = new Sensor();

        waitForStart();
        double dt = System.nanoTime() / 1e9; // initialize delta time
        while (opModeIsActive()) {
            dt = System.nanoTime() / 1e9 - dt; // calculate delta time
            double error = sensor.getError(); // get error
            double output = control.updatePIDF(error, dt); // update the PIDF loop

            // Apply output
            sensor.apply(output);

            // Log data
            telemetry.addData("Error", error);
            telemetry.addData("Output", output);
            telemetry.addData("Delta Time", dt);
            telemetry.update();

        }
    }
}









