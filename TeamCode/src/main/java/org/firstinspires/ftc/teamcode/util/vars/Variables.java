package org.firstinspires.ftc.teamcode.util.vars;

import java.util.HashSet;
import java.util.Set;

public class Variables {
    public static GarbageCollector garbageCollector;
    public static Set<Var<?>> variables = new HashSet<>();
    public static Set<Modifier<?>> modifiers = new HashSet<>();

    public static void add(Var<?> var) {
        variables.add(var);
    }
    public static void finalizeVariable(Var<?> var) {
        garbageCollector.clearModifiers(modifiers, var);
        garbageCollector.collect(var);
        variables.remove(var);
    }

    public static void registerToFTCDashboard(Var<?> var) {
        var.uploadToFtcDashboard();
    }

    public static void registerToFTCDashboard(Modifier<?> modifier) {
        modifier.uploadToFtcDashboard();
    }

    public static void registerAllVariablesToFtcDash() {
        variables.forEach(Var::uploadToFtcDashboard);
    }
    public static void registerAllModifiersToFtcDash() {
        modifiers.forEach(Modifier::uploadToFtcDashboard);
    }
    public static void registerAllToFtcDash() {
        registerAllVariablesToFtcDash();
        registerAllModifiersToFtcDash();
    }
}
