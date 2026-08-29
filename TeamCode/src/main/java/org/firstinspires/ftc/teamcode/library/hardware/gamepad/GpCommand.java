package org.firstinspires.ftc.teamcode.library.hardware.gamepad;

import java.util.Objects;

public abstract class GpCommand {
    public abstract GpCommand copy();

    public enum TYPE {
        ANALOG,
        HYBRID,
        BUTTON
    }

    private final TYPE type;
    private final String name;

    protected GpCommand(TYPE type, String name) {
        this.type = Objects.requireNonNull(type, "type");
        this.name = Objects.requireNonNull(name, "name");
    }

    public TYPE getType() {
        return type;
    }

    public String getName() {
        return name;
    }

    public abstract double getValue();

    @Override
    public final boolean equals(Object other) {
        if (this == other) {
            return true;
        }
        if (other == null || getClass() != other.getClass()) {
            return false;
        }
        GpCommand gpCommand = (GpCommand) other;
        return type == gpCommand.type && name.equals(gpCommand.name);
    }

    @Override
    public final int hashCode() {
        return Objects.hash(type, name, getClass());
    }

    @Override
    public String toString() {
        return name;
    }
}
