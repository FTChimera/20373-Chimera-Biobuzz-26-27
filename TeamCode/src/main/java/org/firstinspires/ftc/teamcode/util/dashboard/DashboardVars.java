package org.firstinspires.ftc.teamcode.util.dashboard;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.ValueProvider;
import com.acmerobotics.dashboard.config.VariableProvider;
import com.rmsftc.utils.config.vars.Modifier;
import com.rmsftc.utils.config.vars.Section;
import com.rmsftc.utils.config.vars.Validator;
import com.rmsftc.utils.config.vars.Var;
import com.rmsftc.utils.config.vars.Variables;

import org.firstinspires.ftc.teamcode.util.SimEnvironment;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.Objects;
import java.util.Set;

/**
 * FTC Dashboard upload for FTCUtils config vars, plus validated var
 * factories.
 *
 * <p>This is the TeamCode-side replacement for the old
 * {@code Var.uploadToFtcDashboard()} (deleted with TeamCode's own vars
 * system in the FTCUtils migration; FTCUtils itself ships no dashboard
 * bridge). The dashboard entries read and write the live vars: changing
 * a value on the dashboard calls {@link Var#set(Object)}, so the var's
 * {@link Validator}s and mutability lock still apply — an out-of-range
 * dashboard edit throws and the value does not change.
 *
 * <p>Note: a modifier's dashboard entry is display-only — it shows the
 * {@code applied} boolean via a {@link VariableProvider}. Nothing wires
 * a dashboard toggle back to {@link Modifier#apply()} or
 * {@link Modifier#revert()}; those are called from code.
 *
 * <p>Typical use in a tuning OpMode:
 * <pre>
 * Var&lt;Integer&gt; speed = DashboardVars.intVar("Shooter Speed", 700, 0, 3000);
 * speed.section.category = "test";
 * DashboardVars.upload(speed);
 * </pre>
 */
public final class DashboardVars {
    private static final Set<Var<?>> uploadedVars = new HashSet<>();
    private static final Set<Modifier<?>> uploadedModifiers = new HashSet<>();

    private DashboardVars() {}

    // ------------------------------------------------------------------
    // Validated var factories. The TypesKt factories (DoubleVar/IntVar/...)
    // take no validators; these attach them. The initial value is not
    // validated (FTCUtils Var only validates on set()).
    // ------------------------------------------------------------------

    /** A named double var that only accepts values in {@code [min, max]}. */
    public static Var<Double> doubleVar(String name, double value, double min, double max) {
        return new Var<>(name, value, Validator.inRange(min, max));
    }

    /** A named int var that only accepts values in {@code [min, max]}. */
    public static Var<Integer> intVar(String name, int value, int min, int max) {
        return new Var<>(name, value, v -> v >= min && v <= max);
    }

    /** A named boolean var (non-null). */
    public static Var<Boolean> boolVar(String name, boolean value) {
        return new Var<>(name, value, Validator.nonnull());
    }

    /** A named string var (non-null). */
    public static Var<String> strVar(String name, String value) {
        return new Var<>(name, value, Validator.nonnull());
    }

    // ------------------------------------------------------------------
    // Dashboard upload.
    // ------------------------------------------------------------------

    /**
     * Uploads a named var to the dashboard under its section (idempotent:
     * a second call does nothing). Throws
     * {@link IllegalArgumentException} for an unnamed var.
     *
     * <p>In sim there is no dashboard (FtcDashboard cannot initialize on the
     * desktop JVM), so this is a no-op — and the var is deliberately NOT
     * marked uploaded, so a later robot-path upload still happens.
     */
    public static <T> void upload(Var<T> var) {
        if (SimEnvironment.isSimulation()) {
            return;
        }
        if (uploadedVars.contains(var)) {
            return;
        }
        upload(var, requireSection(var));
        uploadedVars.add(var);
    }

    /** Uploads a var under an explicit section (not idempotent). No-op in sim. */
    public static <T> void upload(Var<T> var, Section section) {
        if (SimEnvironment.isSimulation()) {
            return;
        }
        FtcDashboard.getInstance().addConfigVariable(section.category, section.name,
                new ValueProvider<T>() {
                    @Override
                    public T get() {
                        return var.get();
                    }

                    @Override
                    public void set(T value) {
                        var.set(value);
                    }
                });
    }

    /**
     * Uploads a modifier to the dashboard (idempotent). The entry is named
     * "<var name> Modifier (val=..., revert=...)" and is recategorized to
     * {@link Section#MODIFIERS_CATEGORY} when the var is in
     * {@link Section#VARIABLES_CATEGORY}. Throws
     * {@link IllegalArgumentException} for a modifier of an unnamed var.
     *
     * <p>No-op in sim (see {@link #upload(Var)}).
     */
    public static <T> void upload(Modifier<T> modifier) {
        if (SimEnvironment.isSimulation()) {
            return;
        }
        if (uploadedModifiers.contains(modifier)) {
            return;
        }
        upload(modifier, modifierSection(modifier));
        uploadedModifiers.add(modifier);
    }

    /** Uploads a modifier under an explicit section (not idempotent). No-op in sim. */
    public static <T> void upload(Modifier<T> modifier, Section section) {
        if (SimEnvironment.isSimulation()) {
            return;
        }
        FtcDashboard.getInstance().addConfigVariable(section.category, section.name,
                new VariableProvider<>(modifier.applied()));
    }

    /** Uploads every registered var and modifier to the dashboard. */
    public static void registerAllToFtcDash() {
        registerAllVariablesToFtcDash();
        registerAllModifiersToFtcDash();
    }

    /** Uploads every registered var to the dashboard. */
    public static void registerAllVariablesToFtcDash() {
        for (Var<?> var : new ArrayList<>(Variables.variables)) {
            upload(var);
        }
    }

    /** Uploads every registered modifier to the dashboard. */
    public static void registerAllModifiersToFtcDash() {
        for (Modifier<?> modifier : new ArrayList<>(Variables.modifiers)) {
            upload(modifier);
        }
    }

    private static <T> Section requireSection(Var<T> var) {
        if (var.section == null) {
            throw new IllegalArgumentException("Only named vars can be uploaded to the dashboard");
        }
        return var.section;
    }

    private static <T> Section modifierSection(Modifier<T> modifier) {
        Var<T> var = modifier.getVar();
        if (var.section == null) {
            throw new IllegalArgumentException(
                    "Only modifiers of named vars can be uploaded to the dashboard");
        }
        Section section = new Section(var.section.category,
                var.section.name + " Modifier (val=" + modifier.getValue()
                        + ", revert=" + modifier.getOriginal() + ")", true);
        if (Objects.equals(var.section.category, Section.VARIABLES_CATEGORY)) {
            section.category = Section.MODIFIERS_CATEGORY;
        }
        return section;
    }
}
