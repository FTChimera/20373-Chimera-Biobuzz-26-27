package org.firstinspires.ftc.teamcode.util.control;

import static org.junit.Assert.assertEquals;

import org.junit.Test;

public class DoubleFlywheelFullVelocityControllerTest {
    private static final double EPSILON = 1e-9;

    @Test
    public void usesMeasuredVoltageWhenValid() {
        DoubleFlywheelFullVelocityController controller =
                new DoubleFlywheelFullVelocityController(constants(true));
        controller.setVoltageSupplier(() -> 10.0);
        controller.setTarget(2.0);

        controller.update(2.0, 0.02);

        assertEquals(0.6, controller.getOutput(), EPSILON);
        assertEquals(0.1, controller.getAppliedVoltageScale(), EPSILON);
    }

    @Test
    public void fallsBackToNominalVoltageWhenMeasuredVoltageIsOutOfRange() {
        DoubleFlywheelFullVelocityController controller =
                new DoubleFlywheelFullVelocityController(constants(true));
        controller.setVoltageSupplier(() -> 8.5);
        controller.setTarget(2.0);

        controller.update(2.0, 0.02);

        assertEquals(0.5, controller.getOutput(), EPSILON);
        assertEquals(1.0 / 12.0, controller.getAppliedVoltageScale(), EPSILON);
    }

    @Test
    public void ignoresVoltageSupplierWhenCompensationDisabled() {
        DoubleFlywheelFullVelocityController controller =
                new DoubleFlywheelFullVelocityController(constants(false));
        controller.setVoltageSupplier(() -> 10.0);
        controller.setTarget(2.0);

        controller.update(2.0, 0.02);

        assertEquals(1.0, controller.getOutput(), EPSILON);
        assertEquals(1.0, controller.getAppliedVoltageScale(), EPSILON);
    }

    private DoubleFlywheelFullVelocityConstants constants(boolean voltageCompensation) {
        return new DoubleFlywheelFullVelocityConstants(
                0.0, 0.0, 0.0,
                0.0, 3.0, 0.0,
                0.0, 0.0,
                voltageCompensation, 12.0
        );
    }
}
