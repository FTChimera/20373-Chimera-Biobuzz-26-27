package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.configs.BlackboardTest;
import org.firstinspires.ftc.teamcode.library.DriveVectorTest;
import org.firstinspires.ftc.teamcode.library.TimerTest;
import org.firstinspires.ftc.teamcode.library.IntakeBallPathTest;
import org.firstinspires.ftc.teamcode.library.control.BangBangControllerTest;
import org.firstinspires.ftc.teamcode.library.control.CompositeControllerTest;
import org.firstinspires.ftc.teamcode.library.control.PIDFControlTest;
import org.firstinspires.ftc.teamcode.library.filters.LowPassFilterTest;
import org.firstinspires.ftc.teamcode.library.game.FieldTest;
import org.firstinspires.ftc.teamcode.library.game.StateTest;
import org.firstinspires.ftc.teamcode.library.hardware.gamepad.GamepadInputTest;
import org.firstinspires.ftc.teamcode.library.interpolation.InterpolationTest;
import org.firstinspires.ftc.teamcode.library.math.EquationMathTest;
import org.firstinspires.ftc.teamcode.library.math.MathUtilsTest;
import org.firstinspires.ftc.teamcode.library.robot.RobotConfigurationTest;
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
        IntakeBallPathTest.class,
        BangBangControllerTest.class,
        CompositeControllerTest.class,
        PIDFControlTest.class,
        LowPassFilterTest.class,
        FieldTest.class,
        StateTest.class,
        GamepadInputTest.class,
        InterpolationTest.class,
        EquationMathTest.class,
        MathUtilsTest.class,
        RobotConfigurationTest.class,
        EquationCurveTest.class
})
public final class UnitTestAll {
    private UnitTestAll() {}
}
