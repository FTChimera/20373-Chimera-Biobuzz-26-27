package org.firstinspires.ftc.teamcode.fakeHw;

/** A physical simulation owner advanced explicitly, never by a wall-clock thread. */
public interface FakeSteppable {
    void advance(double seconds);
}
