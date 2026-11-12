package org.firstinspires.ftc.teamcode.mechanisms;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.firstinspires.ftc.teamcode.util.MatchMode;
import org.junit.Test;

public class RobotParkingTest {
    @Test
    public void qualificationParkingBeginsAtEndgameWarning() {
        assertTrue(Robot.shouldAutomaticallyPark(MatchMode.QUALIFICATIONS, 3.0));
    }

    @Test
    public void qualificationParkingDoesNotStartBeforeEndgameWarning() {
        assertFalse(Robot.shouldAutomaticallyPark(MatchMode.QUALIFICATIONS, 3.01));
    }

    @Test
    public void automaticParkingStopsAtMatchEnd() {
        assertFalse(Robot.shouldAutomaticallyPark(MatchMode.QUALIFICATIONS, 0.0));
    }

    @Test
    public void playoffsDoesNotAutomaticallyPark() {
        assertFalse(Robot.shouldAutomaticallyPark(MatchMode.PLAYOFFS, 3.0));
    }
}
