package org.firstinspires.ftc.teamcode.learning.flyset;


public class Control {
    private double integralSum = 0;
    public double updatePIDF(double error, double dt, double setpoint) {
        // CONSTANTS (Tune for each application)
        double kP = 0.1; // proportional gain
        double kI = 0.01; // integral gain
        double kD = 0.01; // derivative gain
        double kF = 0.2; // feedforward gain

        integralSum += error * dt; // accumulate integral sum

        // Calculate terms
        double proportionalTerm = kP * error; // proportional to error
        double integralTerm = kI * integralSum; // integrate error over time
        double derivativeTerm = kD * (error / dt); // change in error (differentiate error over time)
        double feedforwardTerm = kF * setpoint; // feedforward based on desired setpoint

        double result; // Now we can add all the terms together to get one output
        result = proportionalTerm + integralTerm + derivativeTerm + feedforwardTerm;
        return result;
    }
}
