package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.configs.BlackboardTest;
import org.firstinspires.ftc.teamcode.pedroPathing.custom.EquationCurveTest;
import org.junit.runner.RunWith;
import org.junit.runners.Suite;

/**
 * Single entry point for the TeamCode module JVM unit-test suite.
 *
 * <p>Run this class when you want one target that executes every concrete
 * JUnit test class in TeamCode. Gradle does not expose another module's
 * {@code src/test/java} classes here, so library-side tests stay in the
 * library module's own suite. JUnit's suite runner instantiates each listed
 * test class and runs every method annotated with {@code @Test}.</p>
 */
@RunWith(Suite.class)
@Suite.SuiteClasses({
        BlackboardTest.class,
        EquationCurveTest.class
})
public final class UnitTestAll {
    private UnitTestAll() {}
}
