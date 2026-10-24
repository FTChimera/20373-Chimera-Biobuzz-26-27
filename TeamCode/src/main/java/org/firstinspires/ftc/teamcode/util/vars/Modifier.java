package org.firstinspires.ftc.teamcode.util.vars;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.ValueProvider;
import com.acmerobotics.dashboard.config.VariableProvider;

import java.util.Objects;

public class Modifier<T> {
    volatile Var<T> var;
    T value;
    T original;
    private boolean applied;
    private boolean uploadedToFtcDash = false;
    public Modifier(Var<T> variable, T value) {
        this.var = variable;
        this.value = value;
        this.original = var.get();
    }

    public void apply() {
        this.var.set(value);
        applied = true;
    }
    public void revert() {
        this.var.set(original);
        applied = false;
    }

    public T getOriginal() {
        return original;
    }

    public boolean applied() {
        return applied;
    }

    public Var<T> getVar() {
        return var;
    }

    public void uploadToFtcDashboard() {
        if (uploadedToFtcDash) return;
        Section section = new Section(var.section.category, var.section.name + " Modifier (val=" + value + ", revert=" + original + ")", true);
        if (Objects.equals(var.section.category, Section.VARIABLES_CATEGORY)) {
            section.category = Section.MODIFIERS_CATEGORY;
        }
        uploadToFtcDashboard(section);
        uploadedToFtcDash = true;
    }

    public void uploadToFtcDashboard(Section section) {
        FtcDashboard.getInstance()
                .addConfigVariable(section.category, section.name, new VariableProvider<>(applied));
    }
}
