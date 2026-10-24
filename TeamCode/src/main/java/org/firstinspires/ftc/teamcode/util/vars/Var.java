package org.firstinspires.ftc.teamcode.util.vars;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.ValueProvider;

import java.util.Arrays;

public class Var<T> {
    protected T value;
    protected boolean mutable = true;
    public final Section section;
    protected Validation<T> validation;
    private boolean uploadedToFtcDash = false;

    @SafeVarargs
    public Var(String name, T value, Validator<T>... validators) {
        this.value = value;
        this.section = new Section(Section.VARIABLES_CATEGORY, name, false);
        this.validation = new Validation<>(Arrays.asList(validators));
        Variables.add(this);
    }

    public T get() {
        return value;
    }

    public void set(T value) {
        if (!mutable) {
            throw new IllegalStateException("Cannot modify an immutable Var");
        }
        validation.validate(value);
        this.value = value;
    }

    public void makeMutable() {
        this.mutable = true;
    }

    public boolean isMutable() {
        return mutable;
    }

    public void makeFinal() {
        this.mutable = false;
    }

    public void uploadToFtcDashboard() {
        if (uploadedToFtcDash) {
            return;
        }
        FtcDashboard.getInstance().addConfigVariable(section.category, section.name, new ValueProvider<T>() {

            /**
             * Returns the value.
             */
            @Override
            public T get() {
                return Var.this.get();
            }

            /**
             * Updates the value. {@link #get()} should now return this new value.
             *
             * @param value
             */
            @Override
            public void set(T value) {
                Var.this.set(value);
            }
        });
        uploadedToFtcDash=true;
    }

    public Modifier<T> at(T value) {
        return new Modifier<>(this, value);
    }
}
