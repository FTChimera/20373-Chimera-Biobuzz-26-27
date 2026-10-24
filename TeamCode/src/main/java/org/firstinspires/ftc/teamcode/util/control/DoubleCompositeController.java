package org.firstinspires.ftc.teamcode.util.control;

/**
 * Scalar {@link CompositeController} convenience type.
 */
public class DoubleCompositeController extends CompositeController<Double> {
    public DoubleCompositeController(DoubleCompositeConstants constants) {
        super(constants);
    }
    
    public DoubleCompositeController(
            Controller<Double, Double, Double> nearController,
            Controller<Double, Double, Double> farController,
            double threshold) {
        this(new DoubleCompositeConstants(nearController, farController, threshold));
    }
}
