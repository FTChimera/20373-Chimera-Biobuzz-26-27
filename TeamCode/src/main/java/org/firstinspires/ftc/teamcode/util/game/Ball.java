package org.firstinspires.ftc.teamcode.util.game;

import org.firstinspires.ftc.teamcode.planners.Constants;

public enum Ball {
    POLLEN(Constants.Field.Balls.POLLEN),
    NECTAR(Constants.Field.Balls.NECTAR),
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
