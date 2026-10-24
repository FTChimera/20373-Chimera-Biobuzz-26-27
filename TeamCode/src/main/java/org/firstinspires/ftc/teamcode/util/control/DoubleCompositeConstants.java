package org.firstinspires.ftc.teamcode.util.control;

/**
 * Scalar {@link CompositeConstants} for controllers with {@link Double} state,
 * target, and output values.
 */
public class DoubleCompositeConstants extends CompositeConstants<Double> {
    public DoubleCompositeConstants(
            Controller<Double, Double, Double> nearController,
            Controller<Double, Double, Double> farController,
            double threshold) {
        super(nearController, farController, value -> new double[]{value});
        setThreshold(threshold);
    }
}
