package org.firstinspires.ftc.teamcode.mechanisms;

import com.pedropathing.ivy.Scheduler;
import com.pedropathing.ivy.commands.Commands;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.library.OpModeHelper;
import org.firstinspires.ftc.teamcode.library.game.State;
import org.firstinspires.ftc.teamcode.library.robot.RobotConstants;

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
    private State lastState;
    OpModeHelper helper;

    public Mechanisms(HardwareMap map, RobotConstants bot) {
        super(map, bot);
        helper = new OpModeHelper(hardwareMap);
        bot.Mechanisms.stream()
                .filter(mechanismClass -> mechanismClass == Mechanisms.class)
                .sorted(Comparator.comparing(Class::getName))
                .forEach(mechanismClass -> mechanisms.put(mechanismClass, bot.createMechanism(map, mechanismClass)));
    }

    public void start() {
        helper.start();
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

    public State getLastState() {
        return lastState;
    }

    @Override
    public void periodic(State state) {
        state.setOpModeHelper(helper);
        state.setRobotDrive(require(RobotDrive.class));
        lastState = Objects.requireNonNull(state, "state");
        for (Mechanism mechanism : mechanisms.values()) {
            mechanism.periodic(state);
        }
    }

    public void schedule(Supplier<State> stateSupplier) {
        Objects.requireNonNull(stateSupplier, "State Supplier");
        Scheduler.schedule(Commands.infinite(() -> periodic(stateSupplier.get())));
    }

    // Now create wrapper methods on all the mechanisms
    // like intake full power, etc.
}
