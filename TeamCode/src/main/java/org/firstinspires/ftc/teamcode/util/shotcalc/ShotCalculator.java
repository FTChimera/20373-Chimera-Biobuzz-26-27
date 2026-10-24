package org.firstinspires.ftc.teamcode.util.shotcalc;

@FunctionalInterface
public interface ShotCalculator {
    ShotSolution calculateShot(ShotContext ctx);

    static IdealShotCalc ideal() {
        return new IdealShotCalc();
    }

    static TableShotCalc table() {
        return new TableShotCalc();
    }
}
