package com.rmsftc.library.game;

import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;

import org.junit.Test;

import static org.junit.Assert.assertEquals;

public class FieldTest {
    @Test
    public void shapeContactStateDistinguishesContainedPartialAndOutsideRobotSweeps() {
        assertEquals(
                Field.ContactState.OVERLAPPING,
                Field.Shapes.SOME_CIRCLE.getContactState(new Pose(30, 40), new Pose(), 0.0, 4.0, 4.0)
        );
        assertEquals(
                Field.ContactState.PARTIAL,
                Field.Shapes.SOME_CIRCLE.getContactState(new Pose(39, 40), new Pose(), 0.0, 4.0, 4.0)
        );
        assertEquals(
                Field.ContactState.OUTSIDE,
                Field.Shapes.SOME_CIRCLE.getContactState(new Pose(60, 40), new Pose(), 0.0, 4.0, 4.0)
        );
    }

    @Test
    public void boundaryContactStateUsesVelocitySweepToCatchCrossings() {
        assertEquals(
                Field.ContactState.OVERLAPPING,
                Field.Boundaries.AUTO_MID_LINE_BLUE.getContactState(new Pose(30, 72), new Vector(), 0.25)
        );
        assertEquals(
                Field.ContactState.PARTIAL,
                Field.Boundaries.AUTO_MID_LINE_BLUE.getContactState(new Pose(-10, 72), new Vector(100, 0), 1.0, 6.0, 8.0)
        );
        assertEquals(
                Field.ContactState.OUTSIDE,
                Field.Boundaries.AUTO_MID_LINE_BLUE.getContactState(new Pose(-20, 72), new Vector(), 1.0, 6.0, 8.0)
        );
    }
}
