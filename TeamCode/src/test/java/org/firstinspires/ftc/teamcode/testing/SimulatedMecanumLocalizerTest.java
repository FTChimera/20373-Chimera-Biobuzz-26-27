package org.firstinspires.ftc.teamcode.testing;

import static org.junit.Assert.assertEquals;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.when;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.junit.Test;

public class SimulatedMecanumLocalizerTest {
    @Test
    public void integratesMeasuredWheelVelocityInsteadOfCommandedPower() {
        SimulatedMecanumLocalizer localizer = localizer(500, 500, 500, 500);
        localizer.advance(0.1);
        assertEquals(4, localizer.pose().x(), 1e-12);
        assertEquals(0, localizer.pose().y(), 1e-12);
        assertEquals(40, localizer.velocity().vx, 1e-12);
    }

    @Test
    public void distinguishesStrafeFromTurn() {
        SimulatedMecanumLocalizer strafe = localizer(-500, 500, 500, -500);
        strafe.advance(0.1);
        assertEquals(0, strafe.pose().x(), 1e-12);
        assertEquals(3, strafe.pose().y(), 1e-12);
        SimulatedMecanumLocalizer turn = localizer(-500, 500, -500, 500);
        turn.advance(0.1);
        assertEquals(0, turn.pose().x(), 1e-12);
        assertEquals(0.2, turn.pose().heading(), 1e-12);
    }

    private static SimulatedMecanumLocalizer localizer(double fl, double fr, double bl, double br) {
        return new SimulatedMecanumLocalizer(motor(fl), motor(fr), motor(bl), motor(br),
                1000, 80, 60, 4);
    }

    private static DcMotorEx motor(double velocity) {
        DcMotorEx motor = mock(DcMotorEx.class);
        when(motor.getVelocity()).thenReturn(velocity);
        return motor;
    }
}
