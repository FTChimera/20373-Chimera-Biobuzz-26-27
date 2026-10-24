package org.firstinspires.ftc.teamcode.pedro.commands.fsm;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.CommandBuilder;

import java.util.concurrent.atomic.AtomicBoolean;
import java.util.function.Supplier;

@FunctionalInterface
public interface BasicStateMachine {
    boolean run();

    static BasicStateMachine create(Supplier<Boolean> runFunction) {
        return runFunction::get;
    }

    default CommandBuilder asIvy() {
        AtomicBoolean done = new AtomicBoolean(false);
        return Command.build()
                .requiring(done, this)
                .setExecute(() -> done.set(this.run()))
                .setDone(done::get);
    }
}
