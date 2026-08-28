package org.firstinspires.ftc.teamcode.library.game;

import org.junit.Test;

import static org.junit.Assert.assertEquals;

public class StateTest {
    @Test
    public void stateDefaultsToTeleopMode() {
        assertEquals(RobotMode.TELEOP, new State().getRobotMode());
    }

    @Test
    public void stateAcceptsAnExplicitRobotMode() {
        assertEquals(RobotMode.AUTONOMOUS, new State(RobotMode.AUTONOMOUS).getRobotMode());
    }
}
