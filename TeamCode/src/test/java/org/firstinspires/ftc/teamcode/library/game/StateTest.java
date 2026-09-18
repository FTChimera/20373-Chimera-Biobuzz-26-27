package org.firstinspires.ftc.teamcode.library.game;

import org.firstinspires.ftc.teamcode.library.AllianceColor;
import org.firstinspires.ftc.teamcode.library.Tuple;
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

    @Test
    public void stateCarriesAllianceAndRobotModeTogether() {
        State state = new State(new Tuple<>(AllianceColor.BLUE, RobotMode.AUTONOMOUS));

        assertEquals(AllianceColor.BLUE, state.getAllianceColorAndRobotMode().getA());
        assertEquals(RobotMode.AUTONOMOUS, state.getAllianceColorAndRobotMode().getB());

        state.setAllianceColor(AllianceColor.RED);
        assertEquals(RobotMode.AUTONOMOUS, state.getRobotMode());
        state.setRobotMode(RobotMode.TELEOP);
        assertEquals(AllianceColor.RED, state.getAllianceColor());
    }
}
