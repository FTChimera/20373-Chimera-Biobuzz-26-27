package org.firstinspires.ftc.teamcode.fakeHw;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DigitalChannelController;
import java.util.ArrayList;
import java.util.List;
import java.util.Objects;

/** Input stimulus is separate from output writes; initial input is high. */
public final class FakeDigitalChannel extends FakeSensorDevice implements DigitalChannel {
    private Mode mode = Mode.INPUT;
    private boolean inputState = true;
    private boolean outputState;
    private final List<Boolean> states = new ArrayList<>();
    private final List<Mode> modes = new ArrayList<>();
    @Override public Mode getMode() { requireOpen(); return mode; }
    @Override public void setMode(Mode mode) {
        requireOpen();
        this.mode = Objects.requireNonNull(mode, "mode");
        modes.add(mode);
    }
    @Override public void setMode(DigitalChannelController.Mode mode) {
        Objects.requireNonNull(mode, "mode");
        setMode(Mode.valueOf(mode.name()));
    }
    @Override public boolean getState() { requireOpen(); return mode == Mode.INPUT ? inputState : outputState; }
    public void setInputState(boolean state) { requireOpen(); inputState = state; }
    @Override public void setState(boolean state) {
        requireOpen();
        if (mode != Mode.OUTPUT) throw new IllegalStateException("setState requires OUTPUT mode; use setInputState for stimulus");
        outputState = state;
        states.add(state);
    }
    public List<Boolean> getStateHistory() { return snapshot(states); }
    public List<Mode> getModeHistory() { return snapshot(modes); }
    public int getStateWriteCount() { return states.size(); }
    public int getModeWriteCount() { return modes.size(); }
    @Override public void resetDeviceConfigurationForOpMode() { super.resetDeviceConfigurationForOpMode(); mode = Mode.INPUT; }
}
