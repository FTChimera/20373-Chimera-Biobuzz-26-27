package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.FakeClock;
import org.junit.Test;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.concurrent.atomic.AtomicInteger;

import static org.junit.Assert.*;

public class OpModeRunnerTest {
    private static final long STEP = 10_000_000L;

    private <T extends OpMode> OpModeRunner<T> runner(T mode, int maxTicks) {
        return new OpModeRunner<>(mode, new HardwareMap(null, null),
                new RecordingTelemetry(), new FakeClock(), STEP, maxTicks);
    }

    @Test public void iterativeSequenceCommandsAndPhysicsAreDeterministic() throws Exception {
        TraceMotor motor = new TraceMotor();
        OpModeRunner<Iterative> runner = runner(new Iterative(), 20);
        runner.opMode().hardwareMap.put("drive", motor);
        List<Double> previousCommands = new ArrayList<>();
        runner.startAfterTicks(2).stopAfterTicks(5).beforeTick(r -> {
            previousCommands.add(motor.getPower());
            r.opMode().gamepad1.left_stick_y = -0.75f;
        });

        OpModeRunner.Result result = runner.run();

        assertEquals(OpModeRunner.Outcome.STOP_REQUESTED, result.outcome);
        assertEquals(5, result.ticks);
        assertEquals(5 * STEP, result.elapsedNanos);
        assertEquals(Arrays.asList("init", "init_loop", "init_loop", "start",
                "loop", "loop", "loop", "stop"), runner.opMode().trace);
        assertEquals(Arrays.asList(0.0, 0.0, 0.0, 0.75, 0.75), previousCommands);
        assertEquals(Arrays.asList(0.75, 0.75, 0.75, 0.0), motor.commands);
        assertEquals(0, motor.getPower(), 0);
        assertEquals(5, runner.telemetry().updates().size());
        assertEquals("0.030", runner.telemetry().latest().values().get("runtime"));
        assertNotSame(runner.opMode().gamepad1, runner.opMode().gamepad2);
    }

    @Test public void iterativeStopBeforeStartAndEarlyRequestNeverStart() throws Exception {
        for (boolean duringInit : new boolean[]{false, true}) {
            OpModeRunner<Iterative> runner = runner(new Iterative(), 10);
            runner.opMode().stopInInit = duringInit;
            runner.startAfterTicks(4);
            if (!duringInit) runner.stopAfterTicks(0);
            AtomicInteger cleanup = new AtomicInteger();
            runner.onStop(cleanup::incrementAndGet);
            assertEquals(OpModeRunner.Outcome.STOP_REQUESTED, runner.run().outcome);
            assertEquals(Arrays.asList("init", "stop"), runner.opMode().trace);
            assertEquals(1, cleanup.get());
            assertEquals(0, runner.ticks());
        }
    }

    @Test public void iterativeRequestStopInLoopStopsOnce() throws Exception {
        OpModeRunner<Iterative> runner = runner(new Iterative(), 10);
        runner.opMode().stopInLoop = true;
        assertEquals(OpModeRunner.Outcome.STOP_REQUESTED, runner.run().outcome);
        assertEquals(Arrays.asList("init", "start", "loop", "stop"), runner.opMode().trace);
        assertEquals(1, runner.ticks());
    }

    @Test public void scriptedInputsUpdateRealSdkButtonEdgesAndAliases() throws Exception {
        List<Boolean> pressed = new ArrayList<>();
        List<Boolean> released = new ArrayList<>();
        OpMode mode = new OpMode() {
            @Override public void init() { }
            @Override public void loop() {
                pressed.add(gamepad1.aWasPressed());
                released.add(gamepad1.aWasReleased());
                assertEquals(gamepad1.a, gamepad1.cross);
                assertFalse("SDK edge reads consume the event", gamepad1.aWasPressed());
            }
        };
        OpModeRunner<OpMode> runner = runner(mode, 10).stopAfterTicks(5).beforeTick(r -> {
            r.opMode().gamepad1.a = r.ticks() < 2 || r.ticks() == 3;
        });
        assertEquals(OpModeRunner.Outcome.STOP_REQUESTED, runner.run().outcome);
        assertEquals(Arrays.asList(true, false, false, true, false), pressed);
        assertEquals(Arrays.asList(false, false, true, false, true), released);
    }

    @Test public void linearExecutesFinalUserLogicAndSleepInFixedSteps() throws Exception {
        OpModeRunner<Linear> runner = runner(new Linear(), 20).startAfterTicks(2);
        AtomicInteger cleanup = new AtomicInteger();
        List<Long> clockTrace = new ArrayList<>();
        runner.onStop(cleanup::incrementAndGet).beforeTick(r -> clockTrace.add(r.clock().nowNanos()));
        OpModeRunner.Result result = runner.run();
        assertEquals(OpModeRunner.Outcome.COMPLETED, result.outcome);
        assertEquals(Arrays.asList("init", "start", "loop", "loop", "done"),
                runner.opMode().trace);
        assertEquals(6, result.ticks); // two wait ticks, two active ticks, two rounded sleep ticks
        assertEquals(Arrays.asList(STEP, 2 * STEP, 3 * STEP, 4 * STEP, 5 * STEP, 6 * STEP),
                clockTrace);
        assertEquals(0.04, runner.opMode().finishedRuntime, 1e-12);
        assertEquals(1, cleanup.get());
        assertTrue(runner.stopRequested());
    }

    @Test public void linearStopBeforeStartAndRequestStopAreDifferentFromCompletion() throws Exception {
        OpModeRunner<Linear> early = runner(new Linear(), 10).startAfterTicks(5).stopAfterTicks(2);
        assertEquals(OpModeRunner.Outcome.STOP_REQUESTED, early.run().outcome);
        assertEquals(Arrays.asList("init", "stopped"), early.opMode().trace);

        OpModeRunner<Linear> request = runner(new Linear(), 10);
        request.opMode().requestStopInLoop = true;
        assertEquals(OpModeRunner.Outcome.STOP_REQUESTED, request.run().outcome);
        assertEquals(Arrays.asList("init", "start", "loop", "done"), request.opMode().trace);
        assertEquals(1, request.ticks());
    }

    @Test public void bothModelsTimeOutBoundedlyAndCleanUp() throws Exception {
        for (OpMode mode : new OpMode[]{new Iterative(), new EndlessLinear()}) {
            OpModeRunner<OpMode> runner = runner(mode, 3);
            AtomicInteger cleanup = new AtomicInteger();
            runner.onStop(cleanup::incrementAndGet);
            OpModeRunner.Result result = runner.run();
            assertEquals(OpModeRunner.Outcome.TIMED_OUT, result.outcome);
            assertEquals(3, result.ticks);
            assertEquals(3 * STEP, result.elapsedNanos);
            assertEquals(1, cleanup.get());
            assertTrue(runner.stopRequested());
            if (runner.opMode() instanceof EndlessLinear) {
                assertTrue(((EndlessLinear) runner.opMode()).finallyRan);
            } else assertEquals("stop", ((Iterative) runner.opMode()).trace.get(5));
        }

    }

    @Test public void initWaitAlsoTimesOutAndImmediateStopAbortsUserCode() throws Exception {
        OpModeRunner<Linear> waiting = runner(new Linear(), 3).startAfterTicks(4);
        assertEquals(OpModeRunner.Outcome.TIMED_OUT, waiting.run().outcome);
        assertEquals(Arrays.asList("init"), waiting.opMode().trace);
        AtomicInteger afterStop = new AtomicInteger();
        AtomicInteger cleanup = new AtomicInteger();
        OpModeRunner<LinearOpMode> terminated = this.<LinearOpMode>runner(new LinearOpMode() {
            @Override public void runOpMode() {
                terminateOpModeNow();
                afterStop.incrementAndGet();
            }
        }, 3).onStop(cleanup::incrementAndGet);
        assertEquals(OpModeRunner.Outcome.STOP_REQUESTED, terminated.run().outcome);
        assertEquals(0, afterStop.get());
        assertEquals(1, cleanup.get());
    }

    @Test public void linearInitPollingIdleAndRuntimeResetUseTheSameClock() throws Exception {
        List<String> trace = new ArrayList<>();
        OpModeRunner<LinearOpMode> runner = this.<LinearOpMode>runner(new LinearOpMode() {
            @Override public void runOpMode() {
                while (opModeInInit()) trace.add("init");
                if (!isStopRequested()) {
                    assertTrue(isStarted());
                    assertEquals(0, getRuntime(), 0);
                    idle();
                    assertEquals(0.01, getRuntime(), 1e-12);
                    resetRuntime();
                    sleep(0);
                    assertEquals(0, getRuntime(), 0);
                    assertThrows(IllegalArgumentException.class, () -> sleep(-1));
                    requestOpModeStop();
                }
            }
        }, 10).startAfterTicks(2);
        assertEquals(OpModeRunner.Outcome.STOP_REQUESTED, runner.run().outcome);
        assertEquals(Arrays.asList("init"), trace);
        assertEquals(3, runner.ticks());
    }

    @Test public void exceptionsPropagateWithSuppressedCleanupFailure() throws Exception {
        IllegalStateException original = new IllegalStateException("user failed");
        IllegalArgumentException cleanupFailure = new IllegalArgumentException("cleanup failed");
        OpModeRunner<Iterative> runner = runner(new Iterative(), 5);
        runner.opMode().failure = original;
        AtomicInteger cleanup = new AtomicInteger();
        runner.onStop(() -> { cleanup.incrementAndGet(); throw cleanupFailure; });
        assertSame(original, assertThrows(IllegalStateException.class, runner::run));
        assertArrayEquals(new Throwable[]{cleanupFailure}, original.getSuppressed());
        assertEquals(Arrays.asList("init", "start", "loop", "stop"), runner.opMode().trace);
        assertEquals(1, cleanup.get());
        assertTrue(runner.stopRequested());
        assertThrows(IllegalStateException.class, runner::run);
        assertThrows(IllegalStateException.class, () -> runner.opMode().getRuntime());
        assertThrows(IllegalStateException.class, () -> runner.opMode().loop());
    }

    @Test public void linearExceptionAndInterruptedExceptionAlwaysCleanUp() {
        for (boolean interrupt : new boolean[]{false, true}) {
            AtomicInteger cleanup = new AtomicInteger();
            OpModeRunner<LinearOpMode> runner = this.<LinearOpMode>runner(new LinearOpMode() {
                @Override public void runOpMode() throws InterruptedException {
                    if (interrupt) throw new InterruptedException("user interrupt");
                    throw new IllegalStateException("linear failed");
                }
            }, 5).onStop(cleanup::incrementAndGet);
            if (interrupt) assertThrows(InterruptedException.class, runner::run);
            else assertThrows(IllegalStateException.class, runner::run);
            assertEquals(1, cleanup.get());
            assertTrue(runner.stopRequested());
        }
    }

    @Test public void callbackStopAndCallbackExceptionCleanUp() throws Exception {
        AtomicInteger cleanup = new AtomicInteger();
        OpModeRunner<Iterative> stopped = runner(new Iterative(), 3)
                .beforeTick(OpModeRunner::requestStop).onStop(cleanup::incrementAndGet);
        assertEquals(OpModeRunner.Outcome.STOP_REQUESTED, stopped.run().outcome);
        assertEquals(Arrays.asList("init", "start", "stop"), stopped.opMode().trace);
        assertEquals(1, cleanup.get());

        OpModeRunner<Iterative> failed = runner(new Iterative(), 3)
                .beforeTick(r -> { throw new IllegalStateException("physics failed"); })
                .onStop(cleanup::incrementAndGet);
        assertThrows(IllegalStateException.class, failed::run);
        assertEquals(Arrays.asList("init", "start", "stop"), failed.opMode().trace);
        assertEquals(2, cleanup.get());
    }

    @Test public void unsupportedPollingAndOutsideLifecycleFailClearly() {
        OpModeRunner<LinearOpMode> runner = this.<LinearOpMode>runner(new LinearOpMode() {
            @Override public void runOpMode() {
                while (!isStarted()) { /* Deliberately unsupported non-yielding SDK polling. */ }
            }
        }, 3).startAfterTicks(2);
        IllegalStateException failure = assertThrows(IllegalStateException.class, runner::run);
        assertTrue(failure.getMessage().contains("busy polling"));
        assertThrows(IllegalStateException.class, () -> runner.opMode().waitForStart());
        assertThrows(UnsupportedOperationException.class, () -> runner.opMode().init());
        assertThrows(IllegalStateException.class, () -> runner.opMode().runOpMode());
    }

    @Test public void telemetrySupportsFluentValuesRetentionActionsLinesAndLogs() {
        RecordingTelemetry telemetry = new RecordingTelemetry();
        AtomicInteger value = new AtomicInteger();
        Object action = telemetry.addAction(value::incrementAndGet);
        Telemetry.Line line = telemetry.addLine("row");
        Telemetry.Item computed = line.addData("computed", (Func<Integer>) value::get);
        computed.addData("formatted", "%02d", 3).setCaption("renamed").setRetained(true);
        computed.setValue("%03d", (Func<Integer>) value::get);
        telemetry.addData("discard", 42).setValue("%s", "changed");
        telemetry.log().setCapacity(2);
        telemetry.log().add("old");
        telemetry.log().add("message %d", 1);
        telemetry.log().add("new");
        telemetry.log().setDisplayOrder(Telemetry.Log.DisplayOrder.NEWEST_FIRST);

        assertTrue(telemetry.update());
        RecordingTelemetry.Update first = telemetry.latest();
        assertEquals("001", first.values().get("computed"));
        assertEquals("03", first.values().get("renamed"));
        assertEquals("changed", first.values().get("discard"));
        assertEquals(Arrays.asList("new", "message 1"), first.log);
        assertTrue(telemetry.update());
        assertEquals("002", telemetry.latest().values().get("computed"));
        assertEquals("changed", telemetry.latest().values().get("discard"));
        telemetry.addData("next", 1);
        assertTrue(telemetry.update());
        assertFalse(telemetry.latest().values().containsKey("discard"));
        assertEquals("001", first.values().get("computed"));
        assertTrue(telemetry.removeAction(action));
        assertFalse(telemetry.removeAction(action));
        assertTrue(telemetry.removeItem(computed));
        assertFalse(telemetry.removeItem(computed));
        assertTrue(telemetry.removeLine(line));
        assertThrows(IllegalStateException.class, () -> line.addData("removed", 1));
        assertThrows(UnsupportedOperationException.class, () -> telemetry.speak("hello"));
        assertThrows(UnsupportedOperationException.class, () -> telemetry.setNumDecimalPlaces(1, 2));
    }

    @Test public void telemetryClearAllRemovesActionsButPreservesLog() {
        RecordingTelemetry telemetry = new RecordingTelemetry();
        AtomicInteger calls = new AtomicInteger();
        telemetry.addAction(calls::incrementAndGet);
        telemetry.addData("retained", (Func<Integer>) () -> 5).setRetained(false);
        telemetry.clear();
        telemetry.log().add("log");
        telemetry.update();
        assertEquals(1, calls.get());
        assertTrue(telemetry.latest().entries.isEmpty());
        telemetry.addData("value", 1);
        telemetry.clearAll();
        telemetry.update();
        assertEquals(1, calls.get());
        assertTrue(telemetry.latest().entries.isEmpty());
        assertEquals(Arrays.asList("log"), telemetry.latest().log);
        assertThrows(UnsupportedOperationException.class, () -> telemetry.updates().clear());
    }

    @Test public void cleanupNeverMasksAnExceptionThroughSelfSuppression() {
        IllegalStateException shared = new IllegalStateException("shared");
        OpModeRunner<Iterative> runner = runner(new Iterative() {
            @Override public void stop() { super.stop(); throw shared; }
        }, 3);
        runner.opMode().failure = shared;
        runner.onStop(() -> { throw shared; });
        assertSame(shared, assertThrows(IllegalStateException.class, runner::run));
        assertEquals(0, shared.getSuppressed().length);
        assertEquals(Arrays.asList("init", "start", "loop", "stop"), runner.opMode().trace);
    }

    @Test public void telemetryDeferredClearPreservesMutableItemsAndFluentOrder() {
        RecordingTelemetry telemetry = new RecordingTelemetry();
        Telemetry.Item item = telemetry.addData("first", 1);
        item.addData("last", 3);
        item.addData("middle", 2);
        telemetry.update();
        assertEquals("middle", telemetry.latest().entries.get(1).caption);
        item.setValue(4);
        telemetry.update();
        assertEquals("4", telemetry.latest().values().get("first"));
        assertEquals("3", telemetry.latest().values().get("last"));
        item.addData("replacement", 5);
        telemetry.update();
        assertEquals(1, telemetry.latest().entries.size());
        assertEquals("5", telemetry.latest().values().get("replacement"));
        telemetry.addData("null", (Object) null);
        telemetry.update();
        assertEquals("", telemetry.latest().values().get("null"));
        assertThrows(NullPointerException.class,
                () -> telemetry.addLine("line").addData("null producer", (Func<Object>) null));
    }

    public static class Iterative extends OpMode {
        final List<String> trace = new ArrayList<>();
        TraceMotor motor;
        boolean stopInInit;
        boolean stopInLoop;
        RuntimeException failure;
        @Override public void init() {
            trace.add("init");
            List<TraceMotor> motors = hardwareMap.getAll(TraceMotor.class);
            if (!motors.isEmpty()) motor = motors.get(0);
            if (stopInInit) requestOpModeStop();
        }
        @Override public void init_loop() { trace.add("init_loop"); }
        @Override public void start() { trace.add("start"); }
        @Override public void loop() {
            trace.add("loop");
            if (failure != null) throw failure;
            if (motor != null) motor.setPower(-gamepad1.left_stick_y);
            telemetry.addData("runtime", "%.3f", getRuntime());
            if (stopInLoop) { requestOpModeStop(); requestOpModeStop(); }
        }
        @Override public void stop() {
            trace.add("stop");
            if (motor != null) motor.setPower(0);
        }
    }

    public static class Linear extends LinearOpMode {
        final List<String> trace = new ArrayList<>();
        boolean requestStopInLoop;
        double finishedRuntime;
        @Override public final void runOpMode() {
            trace.add("init");
            waitForStart();
            if (isStopRequested()) { trace.add("stopped"); return; }
            trace.add("start");
            int count = 0;
            while (count < 2 && opModeIsActive()) {
                trace.add("loop");
                count++;
                if (requestStopInLoop) requestOpModeStop();
            }
            sleep(15);
            finishedRuntime = getRuntime();
            trace.add("done");
        }
    }

    public static class EndlessLinear extends LinearOpMode {
        boolean finallyRan;
        @Override public void runOpMode() {
            try {
                waitForStart();
                while (opModeIsActive()) telemetry.addData("tick", getRuntime());
            } finally {
                finallyRan = true;
            }
        }
    }

    /** Small command recorder, independent of the separately developed fake hardware suite. */
    public static class TraceMotor implements DcMotorSimple {
        final List<Double> commands = new ArrayList<>();
        private double power;
        private Direction direction = Direction.FORWARD;
        @Override public void setPower(double value) { power = value; commands.add(value); }
        @Override public double getPower() { return power; }
        @Override public void setDirection(Direction value) { direction = value; }
        @Override public Direction getDirection() { return direction; }
        @Override public Manufacturer getManufacturer() { return Manufacturer.Other; }
        @Override public String getDeviceName() { return "TraceMotor"; }
        @Override public String getConnectionInfo() { return "test"; }
        @Override public int getVersion() { return 1; }
        @Override public void resetDeviceConfigurationForOpMode() { power = 0; }
        @Override public void close() { }
    }
}
