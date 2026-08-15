package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.configs.BlackboardTest;
import com.rmsftc.library.DriveVectorTest;
import com.rmsftc.library.TimerTest;
import com.rmsftc.library.control.PIDFControlTest;
import com.rmsftc.library.filters.LowPassFilterTest;
import com.rmsftc.library.game.FieldTest;
import com.rmsftc.library.hardware.gamepad.GamepadInputTest;
import com.rmsftc.library.interpolation.InterpolationTest;
import com.rmsftc.library.math.EquationMathTest;
import com.rmsftc.library.robot.RobotConfigurationTest;
import org.firstinspires.ftc.teamcode.pedroPathing.custom.EquationCurveTest;
import org.junit.runner.RunWith;
import org.junit.runners.Suite;

/**
 * Single entry point for the TeamCode JVM unit-test suite.
 *
 * <p>Run this class when you want one target that executes every concrete
 * JUnit test class in TeamCode. JUnit's suite runner instantiates each listed
 * test class and runs every method annotated with {@code @Test}.</p>
 */
@RunWith(Suite.class)
@Suite.SuiteClasses({
        BlackboardTest.class,
        DriveVectorTest.class,
        TimerTest.class,
        PIDFControlTest.class,
        LowPassFilterTest.class,
        FieldTest.class,
        GamepadInputTest.class,
        InterpolationTest.class,
        EquationMathTest.class,
        RobotConfigurationTest.class,
        EquationCurveTest.class
})
public final class UnitTestAll {
    private UnitTestAll() {}
}
