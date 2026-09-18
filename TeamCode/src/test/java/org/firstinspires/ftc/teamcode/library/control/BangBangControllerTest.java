package org.firstinspires.ftc.teamcode.library.control;

import org.firstinspires.ftc.teamcode.library.math.MathUtils;
import org.junit.Test;

import static org.junit.Assert.assertArrayEquals;
import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertThrows;
import static org.junit.Assert.assertTrue;

public class BangBangControllerTest {
    @Test
    public void scalarControllerSelectsOutputsByTargetRelationship() {
        DoubleBangBangController controller =
                new DoubleBangBangController(0.75, -0.5, 0.1, 0.05);
        Controller<Double, Double, Double> asController = controller;
        asController.setTarget(10.0);

        assertEquals(0.75, asController.calculate(8.0), MathUtils.EPSILON);
        assertFalse(asController.atTarget());

        assertEquals(-0.5, asController.calculate(12.0), MathUtils.EPSILON);
        assertFalse(asController.atTarget());

        assertEquals(0.1, asController.calculate(10.04), MathUtils.EPSILON);
        assertTrue(asController.atTarget());
        assertEquals(-0.04, controller.getLastError(), MathUtils.EPSILON);
    }

    @Test
    public void genericControllerSelectsEachOutputComponent() {
        BangBangConstants<StateVector> constants = new BangBangConstants<>(
                vector -> vector.values,
                StateVector::new)
                .setOutputs(
                        new StateVector(1.0, 2.0),
                        new StateVector(-3.0, -4.0),
                        new StateVector(0.5, 0.25))
                .setTolerance(new double[]{0.1, 0.2});
        BangBangController<StateVector> controller = new BangBangController<>(constants);
        controller.setTarget(new StateVector(10.0, 10.0));

        controller.update(new StateVector(8.0, 12.0));
        assertArrayEquals(new double[]{1.0, -4.0}, controller.getOutput().values, MathUtils.EPSILON);
        assertArrayEquals(new double[]{2.0, -2.0}, controller.getLastError().values, MathUtils.EPSILON);
        assertFalse(controller.atTarget());

        controller.update(new StateVector(10.1, 9.8));
        assertArrayEquals(new double[]{0.5, 0.25}, controller.getOutput().values, MathUtils.EPSILON);
        assertTrue(controller.atTarget());
    }

    @Test
    public void controllerValidatesConfigurationAndResetState() {
        assertThrows(IllegalArgumentException.class,
                () -> new DoubleBangBangConstants().setTolerance(-0.01));
        assertThrows(IllegalArgumentException.class,
                () -> new DoubleBangBangConstants().setTolerance(new double[]{0.1, -0.1}));
        assertThrows(NullPointerException.class,
                () -> new DoubleBangBangConstants().setOutputs(null, 0.0, 0.0));

        BangBangConstants<Double> incomplete = new BangBangConstants<>(
                value -> new double[]{value},
                values -> values[0]);
        BangBangController<Double> incompleteController = new BangBangController<>(incomplete);
        incompleteController.setTarget(1.0);
        assertThrows(IllegalStateException.class, () -> incompleteController.update(0.0));

        BangBangConstants<Double> missingSuppliers = new BangBangConstants<Double>(null, null)
                .setOutputs(1.0, -1.0, 0.0)
                .setTolerance(new double[]{0.1});
        BangBangController<Double> missingSuppliersController =
                new BangBangController<>(missingSuppliers);
        missingSuppliersController.setTarget(1.0);
        assertThrows(IllegalStateException.class, () -> missingSuppliersController.update(0.0));

        BangBangConstants<StateVector> mismatchedDimensions = new BangBangConstants<>(
                vector -> vector.values,
                StateVector::new)
                .setOutputs(
                        new StateVector(1.0),
                        new StateVector(-1.0),
                        new StateVector(0.0))
                .setTolerance(new double[]{0.1, 0.1});
        BangBangController<StateVector> mismatchedController =
                new BangBangController<>(mismatchedDimensions);
        mismatchedController.setTarget(new StateVector(1.0, 1.0));
        assertThrows(IllegalArgumentException.class,
                () -> mismatchedController.update(new StateVector(0.0, 0.0)));

        DoubleBangBangController controller = new DoubleBangBangController(1.0, 0.1);
        controller.setTarget(5.0);
        controller.update(6.0);
        controller.reset();
        assertFalse(controller.atTarget());
        assertNull(controller.getOutput());
        assertThrows(IllegalStateException.class, controller::getLastError);
    }

    private static final class StateVector {
        private final double[] values;

        private StateVector(double... values) {
            this.values = values;
        }
    }
}
