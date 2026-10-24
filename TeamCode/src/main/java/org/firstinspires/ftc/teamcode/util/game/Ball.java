package org.firstinspires.ftc.teamcode.util.game;

public enum Ball {
    POLLEN(2.8),
    NECTAR(3.6),
    UNKNOWN(0.0);

    private final double diameter;
    Ball(double v) {
        diameter = v;
    }

    public boolean isUnknown() {return this==UNKNOWN;}

    public double getDiameter() {
        return diameter;
    }

    public double getRadius() {
        return diameter/2;
    }
}
