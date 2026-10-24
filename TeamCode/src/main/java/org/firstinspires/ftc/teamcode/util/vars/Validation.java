package org.firstinspires.ftc.teamcode.util.vars;

import java.util.Collection;
import java.util.HashSet;
import java.util.Set;

/** Validation  */
public class Validation<T> {
    public Set<Validator<T>> validators;

    public Validation(Collection<Validator<T>> validators) {
        this.validators = Set.copyOf(validators);
    }

    public void validate(T value) {
        Set<Validator<T>> failed = new HashSet<>();
        for (Validator<T> validator : validators) {
            if (!validator.validate(value)) {
                failed.add(validator);
            }
        }
        if (!failed.isEmpty()) {
            throw new IllegalArgumentException("Validation failed: " + failed);
        }
    }

}
