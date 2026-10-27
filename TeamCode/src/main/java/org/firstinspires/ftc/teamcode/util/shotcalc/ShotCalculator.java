package org.firstinspires.ftc.teamcode.util.shotcalc;

@FunctionalInterface
public interface ShotCalculator {
    ShotSolution calculateShot(ShotContext ctx);
    default String getDebugString() {return "THIS IS A FUNCTIONAL SHOT CALC INTERFACE. (if not, please override this)";}

    static IdealShotCalc ideal() {
        return new IdealShotCalc();
    }

    static TableShotCalc table() {
        return new TableShotCalc();
    }
}
