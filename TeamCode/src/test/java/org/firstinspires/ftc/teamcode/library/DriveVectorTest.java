package org.firstinspires.ftc.teamcode.library;

import org.firstinspires.ftc.teamcode.library.math.MathUtils;
import org.junit.Test;

import static org.junit.Assert.assertArrayEquals;
import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotSame;

public class DriveVectorTest {
    @Test
    public void addScaleAndCopyPreserveHeadingAndIndependence() {
        DriveVector base = new DriveVector(1.0, -2.0, 0.5, 1.2);
        DriveVector sum = base.add(new DriveVector(-0.25, 0.5, 0.25, 0.0));
        DriveVector scaled = base.scale(0.5);
        DriveVector copy = base.copy();

        assertEquals(0.75, sum.forward, MathUtils.EPSILON);
        assertEquals(-1.5, sum.strafe, MathUtils.EPSILON);
        assertEquals(0.75, sum.rotate, MathUtils.EPSILON);
        assertEquals(1.2, sum.robotHeading, MathUtils.EPSILON);

        assertEquals(0.5, scaled.forward, MathUtils.EPSILON);
        assertEquals(-1.0, scaled.strafe, MathUtils.EPSILON);
        assertEquals(0.25, scaled.rotate, MathUtils.EPSILON);
        assertEquals(1.2, scaled.robotHeading, MathUtils.EPSILON);

        assertNotSame(base, copy);
        copy.forward = 99.0;
        assertEquals(1.0, base.forward, MathUtils.EPSILON);
    }

    @Test
    public void fieldCentricRotationAndStickCurveTransformExpectedAxes() {
        DriveVector vector = new DriveVector(1.0, 0.0, -0.5, Math.PI / 2);
        DriveVector rotated = vector.rotateFieldCentric();
        DriveVector curved = vector.wStickCurve(x -> Math.copySign(x * x, x));

        assertEquals(0.0, rotated.forward, MathUtils.EPSILON);
        assertEquals(1.0, rotated.strafe, MathUtils.EPSILON);
        assertEquals(-0.5, rotated.rotate, MathUtils.EPSILON);
        assertEquals(0.0, rotated.robotHeading, MathUtils.EPSILON);

        assertEquals(1.0, curved.forward, MathUtils.EPSILON);
        assertEquals(0.0, curved.strafe, MathUtils.EPSILON);
        assertEquals(-0.25, curved.rotate, MathUtils.EPSILON);
        assertEquals(-0.5, vector.rotate, MathUtils.EPSILON);
    }

    @Test
    public void idleSentinelProducesZeroDrivePowers() {
        double[] powers = DriveVector.IDLE.calculateDrivePowers();
        assertArrayEquals(new double[]{0, 0, 0, 0}, powers, MathUtils.EPSILON);
        assertNotSame(DriveVector.IDLE, DriveVector.IDLE.copy());
    }

    @Test
    public void drivePowersNormalizeWhenMagnitudeExceedsOne() {
        DriveVector vector = new DriveVector(1.0, 1.0, 1.0);

        assertArrayEquals(
                new double[]{1.0, -1.0 / 3.0, 1.0 / 3.0, 1.0 / 3.0},
                vector.calculateDrivePowers(),
                MathUtils.EPSILON
        );
    }
}
