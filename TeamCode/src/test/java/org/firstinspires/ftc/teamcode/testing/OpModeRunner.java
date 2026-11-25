package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.FakeClock;

import java.util.Objects;
import java.util.function.Consumer;

import static org.mockito.ArgumentMatchers.any;
import static org.mockito.ArgumentMatchers.anyLong;
import static org.mockito.Mockito.doAnswer;
import static org.mockito.Mockito.spy;

/**
 * Single-threaded, single-use lifecycle harness. Only SDK runtime/scheduling methods are replaced.
 * Use {@link #opMode()} (Mockito copies instance fields), not the supplied template, for assertions.
 * Run inside FtcSimulationRule when robot code needs its scoped desktop services.
 *
 * <p>A tick advances FakeClock, calls beforeTick, decodes SDK gamepad aliases/edges, then permits user code. Physics therefore
 * integrates the PREVIOUS hardware command. Tick indices are zero based. Start/stop schedules
 * refer to completed ticks; stop wins ties. Runtime starts at init and resets at start.
 * Iterative telemetry is updated after init_loop/loop, as in the SDK.
 *
 * <p>Linear runOpMode executes real user logic on the calling thread. waitForStart, opModeInInit,
 * opModeIsActive, idle, and sleep are cooperative scheduling points. Each active/init poll or idle
 * consumes a tick; sleep rounds up to whole ticks. isStarted/isStopRequested are observational.
 * Repeated observational polling without yielding fails explicitly. Busy loops without SDK calls,
 * Thread.sleep, raw System.nanoTime, additional threads, and SDK internal event-loop hooks are
 * unsupported: the tick limit cannot preempt arbitrary Java code. Inject FakeClock into algorithms.
 */
public final class OpModeRunner<T extends OpMode> {
    public enum Outcome { COMPLETED, STOP_REQUESTED, TIMED_OUT }

    public static final class Result {
        public final Outcome outcome;
        public final int ticks;
        public final long elapsedNanos;

        private Result(Outcome outcome, int ticks, long elapsedNanos) {
            this.outcome = outcome;
            this.ticks = ticks;
            this.elapsedNanos = elapsedNanos;
        }
    }

    private final T opMode;
    private final FakeClock clock;
    private final RecordingTelemetry telemetry;
    private final long stepNanos;
    private final int maxTicks;
    private Consumer<OpModeRunner<T>> beforeTick = runner -> { };
    private Runnable onStop = () -> { };
    private int startTick;
    private int stopTick = Integer.MAX_VALUE;
    private int ticks;
    private int observations;
    private boolean started;
    private boolean stopRequested;
    private boolean timedOut;
    private boolean running;
    private boolean used;
    private boolean cleaning;
    private boolean stepping;
    private long runtimeOrigin;
    private String phase;

    public OpModeRunner(T template, HardwareMap hardwareMap, RecordingTelemetry telemetry,
                        FakeClock clock, long stepNanos, int maxTicks) {
        if (stepNanos <= 0 || maxTicks <= 0) {
            throw new IllegalArgumentException("stepNanos and maxTicks must be positive");
        }
        this.clock = Objects.requireNonNull(clock);
        this.telemetry = Objects.requireNonNull(telemetry);
        this.stepNanos = stepNanos;
        this.maxTicks = maxTicks;
        opMode = spy(Objects.requireNonNull(template));
        opMode.hardwareMap = Objects.requireNonNull(hardwareMap);
        opMode.telemetry = telemetry;
        opMode.gamepad1 = new Gamepad();
        opMode.gamepad2 = new Gamepad();
        installRuntime();
    }

    public T opMode() { return opMode; }
    public FakeClock clock() { return clock; }
    public RecordingTelemetry telemetry() { return telemetry; }
    public int ticks() { return ticks; }
    public boolean started() { return started; }
    public boolean stopRequested() { return stopRequested; }

    public OpModeRunner<T> startAfterTicks(int count) {
        configure();
        if (count < 0) throw new IllegalArgumentException("Negative start tick");
        startTick = count;
        return this;
    }
    public OpModeRunner<T> stopAfterTicks(int count) {
        configure();
        if (count < 0) throw new IllegalArgumentException("Negative stop tick");
        stopTick = count;
        return this;
    }
    public OpModeRunner<T> beforeTick(Consumer<OpModeRunner<T>> callback) {
        configure();
        beforeTick = Objects.requireNonNull(callback);
        return this;
    }
    /** Called exactly once during cleanup, even after timeout or user exceptions. */
    public OpModeRunner<T> onStop(Runnable callback) {
        configure();
        onStop = Objects.requireNonNull(callback);
        return this;
    }
    public void requestStart() {
        active();
        if (!stopRequested && !started) {
            started = true;
            runtimeOrigin = clock.nowNanos();
            opMode.time = 0;
            opMode.gamepad1.resetEdgeDetection();
            opMode.gamepad2.resetEdgeDetection();
            processGamepads();
        }
    }
    public void requestStop() {
        active();
        stopRequested = true;
    }

    public Result run() throws InterruptedException {
        configure();
        used = true;
        running = true;
        long origin = clock.nowNanos();
        runtimeOrigin = origin;
        Outcome outcome = Outcome.COMPLETED;
        Throwable failure = null;
        try {
            if (opMode instanceof LinearOpMode) {
                phase = "runOpMode";
                ((LinearOpMode) opMode).runOpMode();
            } else {
                runIterative();
            }
            if (timedOut) outcome = Outcome.TIMED_OUT;
            else if (stopRequested) outcome = Outcome.STOP_REQUESTED;
        } catch (TickLimitReached timeout) {
            outcome = Outcome.TIMED_OUT;
        } catch (ImmediateStop stop) {
            outcome = Outcome.STOP_REQUESTED;
        } catch (InterruptedException | RuntimeException | Error exception) {
            failure = exception;
            throw exception;
        } finally {
            stopRequested = true;
            cleaning = true;
            phase = "stop";
            Throwable cleanupFailure = null;
            try {
                opMode.stop();
            } catch (RuntimeException | Error exception) {
                cleanupFailure = exception;
            }
            try {
                onStop.run();
            } catch (RuntimeException | Error exception) {
                if (cleanupFailure == null) cleanupFailure = exception;
                else if (cleanupFailure != exception) cleanupFailure.addSuppressed(exception);
            } finally {
                running = false;
                cleaning = false;
            }
            if (cleanupFailure != null) {
                if (failure != null) {
                    if (failure != cleanupFailure) failure.addSuppressed(cleanupFailure);
                }
                else if (cleanupFailure instanceof RuntimeException) {
                    throw (RuntimeException) cleanupFailure;
                } else throw (Error) cleanupFailure;
            }
        }
        return new Result(outcome, ticks, clock.nowNanos() - origin);
    }

    private void runIterative() {
        phase = "init";
        opMode.init();
        signals();
        while (!started && !stopRequested) {
            tick();
            if (!started && !stopRequested) {
                phase = "init_loop";
                opMode.init_loop();
                telemetry.update();
            }
            signals();
        }
        if (stopRequested) return;
        phase = "start";
        opMode.start();
        while (!stopRequested) {
            signals();
            if (stopRequested) break;
            tick();
            if (!stopRequested) {
                phase = "loop";
                opMode.loop();
                telemetry.update();
            }
        }
    }

    private void tick() {
        active();
        if (cleaning || stepping) throw new IllegalStateException("Cannot step during callback/cleanup");
        signals();
        if (stopRequested) return;
        if (ticks == maxTicks) {
            timedOut = true;
            stopRequested = true;
            throw new TickLimitReached();
        }
        observations = 0;
        stepping = true;
        try {
            clock.add(stepNanos);
            opMode.time = runtime();
            beforeTick.accept(this);
            processGamepads();
            ticks++;
        } finally {
            stepping = false;
        }
    }

    private void signals() {
        if (ticks >= stopTick) stopRequested = true;
        if (ticks >= startTick && !stopRequested) requestStart();
    }
    private void processGamepads() {
        // SDK decoding updates button aliases and edge queues independently of the public fields.
        opMode.gamepad1.copy(opMode.gamepad1);
        opMode.gamepad2.copy(opMode.gamepad2);
    }
    private double runtime() { return (clock.nowNanos() - runtimeOrigin) / 1e9; }
    private void active() {
        if (!running) throw new IllegalStateException("SDK lifecycle method used outside runner.run()");
    }
    private void configure() {
        if (used) throw new IllegalStateException("OpModeRunner is single-use");
    }
    private void observe() {
        active();
        signals();
        if (++observations > 10_000) {
            throw new IllegalStateException("Unsupported busy polling: yield with idle/sleep or active/init poll");
        }
    }

    private void installRuntime() {
        doAnswer(call -> { active(); return runtime(); }).when(opMode).getRuntime();
        doAnswer(call -> { active(); runtimeOrigin = clock.nowNanos(); return null; })
                .when(opMode).resetRuntime();
        doAnswer(call -> { requestStop(); return null; }).when(opMode).requestOpModeStop();
        doAnswer(call -> { requestStop(); throw new ImmediateStop(); }).when(opMode).terminateOpModeNow();
        doAnswer(call -> {
            active();
            ((org.firstinspires.ftc.robotcore.external.Telemetry) call.getArgument(0)).update();
            return null;
        }).when(opMode).updateTelemetry(any());
        doAnswer(call -> {
            throw new UnsupportedOperationException("SDK internal telemetry transport is unsupported");
        }).when(opMode).internalUpdateTelemetryNow(any());
        doAnswer(call -> { throw new UnsupportedOperationException("SDK internal lifecycle hook"); })
                .when(opMode).internalPreInit();
        doAnswer(call -> { throw new UnsupportedOperationException("SDK internal lifecycle hook"); })
                .when(opMode).internalPostInitLoop();
        doAnswer(call -> { throw new UnsupportedOperationException("SDK internal lifecycle hook"); })
                .when(opMode).internalPostLoop();
        if (!(opMode instanceof LinearOpMode)) {
            doAnswer(call -> { lifecycle("init"); return call.callRealMethod(); }).when(opMode).init();
            doAnswer(call -> { lifecycle("init_loop"); return call.callRealMethod(); })
                    .when(opMode).init_loop();
            doAnswer(call -> { lifecycle("start"); return call.callRealMethod(); }).when(opMode).start();
            doAnswer(call -> { lifecycle("loop"); return call.callRealMethod(); }).when(opMode).loop();
            doAnswer(call -> { lifecycle("stop"); return call.callRealMethod(); }).when(opMode).stop();
            return;
        }
        LinearOpMode linear = (LinearOpMode) opMode;
        try {
            doAnswer(call -> { lifecycle("runOpMode"); return call.callRealMethod(); })
                    .when(linear).runOpMode();
        } catch (InterruptedException impossibleDuringStubbing) {
            throw new AssertionError(impossibleDuringStubbing);
        }
        doAnswer(call -> {
            active();
            signals();
            while (!started && !stopRequested) { tick(); signals(); }
            return null;
        }).when(linear).waitForStart();
        doAnswer(call -> { observe(); return started; }).when(linear).isStarted();
        doAnswer(call -> { observe(); return stopRequested; }).when(linear).isStopRequested();
        doAnswer(call -> {
            active();
            signals();
            if (started && !stopRequested) { tick(); signals(); }
            return started && !stopRequested;
        }).when(linear).opModeIsActive();
        doAnswer(call -> {
            active();
            signals();
            if (!started && !stopRequested) { tick(); signals(); }
            return !started && !stopRequested;
        }).when(linear).opModeInInit();
        doAnswer(call -> { tick(); signals(); return null; }).when(linear).idle();
        doAnswer(call -> {
            active();
            long milliseconds = call.getArgument(0);
            if (milliseconds < 0) throw new IllegalArgumentException("Negative sleep duration");
            long duration = Math.multiplyExact(milliseconds, 1_000_000L);
            long count = duration / stepNanos + (duration % stepNanos == 0 ? 0 : 1);
            for (long i = 0; i < count && !stopRequested; i++) { tick(); signals(); }
            return null;
        }).when(linear).sleep(anyLong());
        doAnswer(call -> { throw new UnsupportedOperationException("Linear lifecycle uses runOpMode"); })
                .when(linear).init();
        doAnswer(call -> { throw new UnsupportedOperationException("Linear lifecycle uses runOpMode"); })
                .when(linear).init_loop();
        doAnswer(call -> { throw new UnsupportedOperationException("Linear lifecycle uses runOpMode"); })
                .when(linear).start();
        doAnswer(call -> { throw new UnsupportedOperationException("Linear lifecycle uses runOpMode"); })
                .when(linear).loop();
        doAnswer(call -> {
            if (!cleaning) throw new UnsupportedOperationException("Only runner cleanup may call stop");
            return null;
        }).when(linear).stop();
    }

    private static final class TickLimitReached extends Error {
        private static final long serialVersionUID = 1L;
    }

    private static final class ImmediateStop extends Error {
        private static final long serialVersionUID = 1L;
    }

    private void lifecycle(String expected) {
        active();
        if (!expected.equals(phase) || stepping) {
            throw new IllegalStateException("Unsupported lifecycle callback: " + expected);
        }
    }
}
