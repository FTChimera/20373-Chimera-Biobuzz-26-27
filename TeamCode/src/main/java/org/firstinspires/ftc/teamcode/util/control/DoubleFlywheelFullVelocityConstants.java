package org.firstinspires.ftc.teamcode.util.control;

public class DoubleFlywheelFullVelocityConstants {
    public double kP;
    public double kI;
    public double kD;
    public double kS;
    public double kV;
    public double kA;

    public double tolerance;
    public double integralLimit;

    public DoubleFlywheelFullVelocityConstants(double p, double i, double d, double s, double v, double a, double tolerance, double integralLimit) {
        this.kP = p;
        this.kI = i;
        this.kD = d;
        this.kS = s;
        this.kV = v;
        this.kA = a;
        this.tolerance = tolerance;
        this.integralLimit = integralLimit;
    }
}
