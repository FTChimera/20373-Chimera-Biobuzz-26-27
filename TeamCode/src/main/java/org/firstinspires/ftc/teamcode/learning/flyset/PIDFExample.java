package org.firstinspires.ftc.teamcode.learning.flyset;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class PIDFExample extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Control control = new Control(); // Our PIDF Controller Mechanism
        Sensor sensor = new Sensor(); // Sensor for getting error
        Actuator actuator = new Actuator(); // Actuator to move using controller.
        double setpoint = 0;
        waitForStart();
        double lastTime = System.nanoTime();
        double dt = 0; // initialize delta time
        while (opModeIsActive()) {
            double currentTime = System.nanoTime(); // Current time
            // Delta time = current time - last time, converted to seconds
            dt = (currentTime - lastTime) / 1e9; // calculate delta time
            lastTime = currentTime;
            double error = sensor.getError(); // get error
            double output = control.updatePIDF(error, dt, setpoint); // update the PIDF loop

            // Apply output
            actuator.apply(output);

            // Log data
            telemetry.addData("Error", error);
            telemetry.addData("Output", output);
            telemetry.addData("Delta Time", dt);
            telemetry.update();

        }
    }
}








