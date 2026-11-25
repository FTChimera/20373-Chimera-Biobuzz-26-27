package org.firstinspires.ftc.teamcode.mechanisms;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotNull;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.rmsftc.utils.config.vars.Modifier;
import com.rmsftc.utils.config.vars.Section;
import com.rmsftc.utils.config.vars.Var;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.SimEnvironment;
import org.firstinspires.ftc.teamcode.util.dashboard.DashboardVars;
import org.junit.AfterClass;
import org.junit.Test;

import java.lang.reflect.InvocationHandler;
import java.lang.reflect.Method;
import java.lang.reflect.Proxy;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * Regression test for the ftcsim FtcDashboard crash: on a desktop JVM the
 * dashboard class's static initializer eagerly builds the singleton, whose
 * constructor needs an Android Activity (AppUtil.getInstance().getActivity()
 * is null), so the constructor throws, poisoning the class for the whole JVM
 * — every later use then fails with NoClassDefFoundError: Could not
 * initialize class com.acmerobotics.dashboard.FtcDashboard.
 *
 * <p>Both production touches of the dashboard go through the seams tested
 * here: the Robot constructor's telemetry wrap ({@link Robot#buildTelemetry})
 * and {@link DashboardVars#upload} (called by the Robot constructor for the
 * SOTM flag and by MainTeleOp for the auto-fire tolerance). A full
 * {@code new Robot(...)} is deliberately NOT constructed here: it needs drive
 * motors plus a Pinpoint (concrete, hardware-touching classes), which is too
 * heavy for a unit test — the two seams above are exactly the code the
 * constructor runs for its dashboard wiring, so they cover the crash path.
 *
 * <p>Load-order trap (same as OpModeStorageSimTest): FtcDashboard's class
 * initializer runs once, at first class load, so the sim override must be
 * set BEFORE any reference to FtcDashboard in the whole suite. That is why
 * the override lives in this class's static initializer (runs at class-load
 * time, before any test method) and why this class is listed right after
 * OpModeStorageSimTest in UnitTestAll. No other test references FtcDashboard
 * (verified), so nothing can trigger the first load earlier.
 */
public class RobotDashboardSimTest {

    static {
        // Must run before FtcDashboard first loads (see class javadoc).
        SimEnvironment.setOverride(true);
    }

    @AfterClass
    public static void restoreAutoDetect() {
        SimEnvironment.setOverride(null);
    }

    @Test
    public void dashboardVarsUploadIsNoopInSim() {
        // Without the guard, either call touches FtcDashboard and this test
        // errors with NoClassDefFoundError.
        Var<Boolean> var = DashboardVars.boolVar("Robot dashboard sim test var", false);
        DashboardVars.upload(var);
        DashboardVars.upload(var, new Section("test", "Robot dashboard sim test var", false));
    }

    @Test
    public void dashboardVarsModifierUploadIsNoopInSim() {
        Var<Boolean> var = DashboardVars.boolVar("Robot dashboard sim test modifier var", false);
        Modifier<Boolean> modifier = new Modifier<>(var, true);
        DashboardVars.upload(modifier);
        DashboardVars.upload(modifier, new Section("test", "Robot dashboard sim test modifier", true));
    }

    @Test
    public void dashboardVarsRegisterAllIsNoopInSim() {
        DashboardVars.registerAllToFtcDash();
    }

    @Test
    public void robotTelemetryIsPlainTelemetryInSim() {
        // Without the guard, buildTelemetry touches FtcDashboard and this
        // test errors with NoClassDefFoundError. With it, the result wraps
        // only the given telemetry: calls forward to it and nowhere else.
        RecordingTelemetry recording = new RecordingTelemetry();
        Telemetry proxy = (Telemetry) Proxy.newProxyInstance(
                RobotDashboardSimTest.class.getClassLoader(),
                new Class<?>[]{Telemetry.class},
                recording);
        MultipleTelemetry built = Robot.buildTelemetry(proxy);
        assertNotNull(built);
        // The constructor itself calls log() on the wrapped telemetry (to
        // build the merged log), then the calls below forward to it.
        built.addData("key", "value");
        built.update();
        assertEquals(Arrays.asList("log", "addData", "update"), recording.calls);
    }

    /** A Telemetry stand-in that records every method call and returns defaults. */
    private static final class RecordingTelemetry implements InvocationHandler {
        final List<String> calls = new ArrayList<>();

        @Override
        public Object invoke(Object proxy, Method method, Object[] args) {
            calls.add(method.getName());
            Class<?> returnType = method.getReturnType();
            if (!returnType.isPrimitive()) {
                return null;
            }
            if (returnType == boolean.class) {
                return false;
            }
            if (returnType == char.class) {
                return (char) 0;
            }
            if (returnType == byte.class) {
                return (byte) 0;
            }
            if (returnType == short.class) {
                return (short) 0;
            }
            if (returnType == int.class) {
                return 0;
            }
            if (returnType == long.class) {
                return 0L;
            }
            if (returnType == float.class) {
                return 0f;
            }
            return 0.0;
        }
    }
}
