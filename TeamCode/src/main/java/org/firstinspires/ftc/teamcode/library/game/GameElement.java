package org.firstinspires.ftc.teamcode.library.game;

public abstract class GameElement {
    public abstract double getDiameter();
    public double getRadius() {return getDiameter()/2;}
}
