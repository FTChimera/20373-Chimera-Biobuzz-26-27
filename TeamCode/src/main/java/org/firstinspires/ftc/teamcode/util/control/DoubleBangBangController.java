package org.firstinspires.ftc.teamcode.util.control;

/**
 * One-dimensional convenience controller for scalar controls.
 */
public class DoubleBangBangController extends BangBangController<Double> {
    public DoubleBangBangController(DoubleBangBangConstants constants) {
        super(constants);
    }

    public DoubleBangBangController(
            double belowOutput, double aboveOutput, double sameOutput, double tolerance) {
        this(new DoubleBangBangConstants(belowOutput, aboveOutput, sameOutput, tolerance));
    }

    public DoubleBangBangController(double outputMagnitude) {
        this(outputMagnitude, 0.0);
    }

    public DoubleBangBangController(double outputMagnitude, double tolerance) {
        this(validateOutputMagnitude(outputMagnitude), -outputMagnitude, 0.0, tolerance);
    }

    private static double validateOutputMagnitude(double outputMagnitude) {
        if (outputMagnitude < 0.0) {
            throw new IllegalArgumentException("outputMagnitude must be non-negative.");
        }
        return outputMagnitude;
    }
}
