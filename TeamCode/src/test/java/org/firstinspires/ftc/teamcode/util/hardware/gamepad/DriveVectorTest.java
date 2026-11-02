package org.firstinspires.ftc.teamcode.util.hardware.gamepad;

import static org.junit.Assert.assertEquals;

import org.junit.Test;

public class DriveVectorTest {
    private static final double EPSILON = 1e-9;

    @Test
    public void fieldCentricUsesRadians() {
        DriveVector vector = new DriveVector(1.0, 0.0, 0.0);

        vector.fieldCentric(Math.PI / 2.0);

        assertEquals(0.0, vector.forward(), EPSILON);
        assertEquals(-1.0, vector.strafe(), EPSILON);
    }

    @Test
    public void headingConstructorUsesRadians() {
        DriveVector vector = new DriveVector(0.0, 1.0, 0.0, Math.PI / 2.0);

        assertEquals(1.0, vector.forward(), EPSILON);
        assertEquals(0.0, vector.strafe(), EPSILON);
    }
}
