package org.firstinspires.ftc.teamcode.util.vars;

import java.util.Set;

public class GarbageCollector {
    public GarbageCollector() {}
    public void collect(Var<?> var) {
        var.set(null);
        var.makeFinal();
    }

    public void clearModifiers(Set<Modifier<?>> modifiers, Var<?> var) {
        modifiers.stream().filter(modifier -> modifier.getVar() == var).forEach(modifiers::remove);
    }
}
