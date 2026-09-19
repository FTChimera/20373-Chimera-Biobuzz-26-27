package org.firstinspires.ftc.teamcode.library.control;

import org.junit.Test;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertThrows;
import static org.junit.Assert.assertTrue;

public class CompositeControllerTest {
    @Test
    public void scalarControllerSelectsNearControllerAtOrInsideThreshold() {
        RecordingController near = new RecordingController(0.2, true);
        RecordingController far = new RecordingController(0.8, false);
        DoubleCompositeController controller = new DoubleCompositeController(near, far, 1.0);
        controller.setTarget(10.0);

        controller.update(9.0);
        assertEquals(0.2, controller.getOutput(), 0.0);
        assertTrue(controller.isUsingNearController());
        assertTrue(controller.atTarget());

        controller.update(8.99);
        assertEquals(0.8, controller.getOutput(), 0.0);
        assertFalse(controller.isUsingNearController());
        assertFalse(controller.atTarget());

        assertEquals(2, near.updateCount);
        assertEquals(2, far.updateCount);
    }

    @Test
    public void resetAndTargetPropagateToBothControllers() {
        RecordingController near = new RecordingController(0.2, true);
        RecordingController far = new RecordingController(0.8, false);
        DoubleCompositeController controller = new DoubleCompositeController(near, far, 1.0);

        controller.setTarget(5.0);
        controller.update(5.0);
        controller.reset();

        assertEquals(Double.valueOf(5.0), near.target);
        assertEquals(Double.valueOf(5.0), far.target);
        assertEquals(1, near.resetCount);
        assertEquals(1, far.resetCount);
        assertFalse(controller.atTarget());
    }

    @Test
    public void controllerValidatesThresholdAndDimensions() {
        RecordingController near = new RecordingController(0.2, true);
        RecordingController far = new RecordingController(0.8, false);
        assertThrows(IllegalArgumentException.class,
                () -> new DoubleCompositeConstants(near, far, -0.1));

        CompositeConstants<Vector> constants = new CompositeConstants<>(
                new VectorController(), new VectorController(), vector -> vector.values)
                .setThreshold(new double[]{1.0});
        CompositeController<Vector> controller = new CompositeController<>(constants);
        controller.setTarget(new Vector(0.0, 0.0));
        assertThrows(IllegalArgumentException.class, () -> controller.update(new Vector(0.0, 0.0)));
    }

    private static final class RecordingController implements Controller<Double, Double, Double> {
        private final double output;
        private final boolean atTarget;
        private Double target;
        private int updateCount;
        private int resetCount;

        private RecordingController(double output, boolean atTarget) {
            this.output = output;
            this.atTarget = atTarget;
        }

        @Override
        public void update(Double state) {
            updateCount++;
        }

        @Override
        public void reset() {
            resetCount++;
        }

        @Override
        public void setTarget(Double target) {
            this.target = target;
        }

        @Override
        public Double getTarget() {
            return target;
        }

        @Override
        public Double getOutput() {
            return output;
        }

        @Override
        public boolean atTarget() {
            return atTarget;
        }
    }

    private static final class Vector {
        private final double[] values;

        private Vector(double... values) {
            this.values = values;
        }
    }

    private static final class VectorController implements Controller<Vector, Vector, Vector> {
        private Vector target;

        @Override
        public void update(Vector state) {}

        @Override
        public void reset() {}

        @Override
        public void setTarget(Vector target) {
            this.target = target;
        }

        @Override
        public Vector getTarget() {
            return target;
        }

        @Override
        public Vector getOutput() {
            return null;
        }

        @Override
        public boolean atTarget() {
            return false;
        }
    }
}
