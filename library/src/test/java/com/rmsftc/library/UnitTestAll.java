package com.rmsftc.library;

import com.rmsftc.ExampleUnitTest;
import com.rmsftc.library.control.PIDFControlTest;
import com.rmsftc.library.filters.LowPassFilterTest;
import com.rmsftc.library.game.FieldTest;
import com.rmsftc.library.hardware.gamepad.GamepadInputTest;
import com.rmsftc.library.interpolation.InterpolationTest;
import com.rmsftc.library.math.EquationMathTest;
import com.rmsftc.library.robot.RobotConfigurationTest;
import org.junit.runner.RunWith;
import org.junit.runners.Suite;

/**
 * Single entry point for the library module JVM unit-test suite.
 */
@RunWith(Suite.class)
@Suite.SuiteClasses({
        ExampleUnitTest.class,
        DriveVectorTest.class,
        TimerTest.class,
        PIDFControlTest.class,
        LowPassFilterTest.class,
        FieldTest.class,
        GamepadInputTest.class,
        InterpolationTest.class,
        EquationMathTest.class,
        RobotConfigurationTest.class    
})
public final class UnitTestAll {
    private UnitTestAll() {}
}
