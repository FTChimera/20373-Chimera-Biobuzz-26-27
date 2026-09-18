package org.firstinspires.ftc.teamcode.library.control;

/**
 * One-dimensional {@link BangBangConstants} for scalar controls.
 */
public class DoubleBangBangConstants extends BangBangConstants<Double> {
    public DoubleBangBangConstants() {
        super(value -> new double[]{value}, DoubleBangBangConstants::fromArray);
    }

    public DoubleBangBangConstants(
            double belowOutput, double aboveOutput, double sameOutput, double tolerance) {
        this();
        setOutputs(belowOutput, aboveOutput, sameOutput);
        setTolerance(tolerance);
    }

    private static Double fromArray(double[] values) {
        if (values == null || values.length != 1) {
            throw new IllegalArgumentException("Expected exactly one scalar value.");
        }
        return values[0];
    }
}
