package org.firstinspires.ftc.teamcode.mechanisms;

import com.pedropathing.ivy.Scheduler;
import com.pedropathing.ivy.commands.Commands;
import com.qualcomm.robotcore.hardware.HardwareMap;

import com.rmsftc.library.hardware.gamepad.GpBindings;
import com.rmsftc.library.robot.RobotConstants;

import java.util.Collection;
import java.util.Collections;
import java.util.Comparator;
import java.util.LinkedHashMap;
import java.util.Map;
import java.util.Objects;
import java.util.Set;
import java.util.function.Supplier;

/**
 * Aggregate access to every profile-managed mechanism registered on a robot profile.
 */
public class Mechanisms extends Mechanism {
    private final Map<Class<? extends Mechanism>, Mechanism> mechanisms = new LinkedHashMap<>();
    private GpBindings lastGpBindings;

    public Mechanisms(HardwareMap map, RobotConstants bot) {
        super(map, bot);
        bot.Mechanisms.stream()
                .filter(mechanismClass -> mechanismClass != Mechanisms.class)
                .sorted(Comparator.comparing(Class::getName))
                .forEach(mechanismClass -> mechanisms.put(mechanismClass, bot.createMechanism(map, mechanismClass)));
    }

    public Set<Class<? extends Mechanism>> getConfiguredMechanismClasses() {
        return Collections.unmodifiableSet(mechanisms.keySet());
    }

    public Collection<Mechanism> all() {
        return Collections.unmodifiableCollection(mechanisms.values());
    }

    public boolean has(Class<? extends Mechanism> mechanismClass) {
        return mechanisms.containsKey(mechanismClass);
    }

    public <M extends Mechanism> M get(Class<M> mechanismClass) {
        Mechanism mechanism = mechanisms.get(mechanismClass);
        return mechanism == null ? null : mechanismClass.cast(mechanism);
    }

    public <M extends Mechanism> M require(Class<M> mechanismClass) {
        M mechanism = get(mechanismClass);
        if (mechanism == null) {
            throw new IllegalStateException("Mechanism not aggregated: " + mechanismClass.getSimpleName());
        }
        return mechanism;
    }

    public GpBindings getLastGpBindings() {
        return lastGpBindings;
    }

    @Override
    public void periodic(GpBindings GpBindings) {
        lastGpBindings = Objects.requireNonNull(GpBindings, "GpBindings");
        for (Mechanism mechanism : mechanisms.values()) {
            mechanism.periodic(GpBindings);
        }
    }

    public void schedule(Supplier<GpBindings> GpBindingsSupplier) {
        Objects.requireNonNull(GpBindingsSupplier, "GpBindingsSupplier");
        Scheduler.schedule(Commands.infinite(() -> periodic(GpBindingsSupplier.get())));
    }
}
