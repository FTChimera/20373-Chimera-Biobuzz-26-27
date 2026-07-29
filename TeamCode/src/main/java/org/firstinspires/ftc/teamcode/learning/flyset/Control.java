package org.firstinspires.ftc.teamcode.learning.flyset;

import org.firstinspires.ftc.teamcode.library.control.DoublePIDFCoeffs;
import org.firstinspires.ftc.teamcode.library.control.DoublePIDFController;

public class Control {
    private DoublePIDFController pidf = new DoublePIDFController(new DoublePIDFCoeffs(0.1, 0.01, 0.01, 0.2));
    public double updatePIDF(double error, double dt) {
        pidf.update(error);
        return pidf.getOutput();
    }
}
