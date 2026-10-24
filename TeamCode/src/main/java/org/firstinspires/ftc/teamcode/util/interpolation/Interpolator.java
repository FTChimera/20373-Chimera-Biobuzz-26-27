package org.firstinspires.ftc.teamcode.util.interpolation;

@FunctionalInterface
public interface Interpolator {
    double interpolate(double input);

    Interpolator identity = input -> input;
    static Interpolator constant(double constant) {
        return input -> constant;
    }
    static Interpolator linear(double x1, double y1, double x2, double y2) {
        return input -> {
            double slope = (y2 - y1) / (x2 - x1);
            return y1 + slope * (input - x1);
        };
    }
    static Interpolator piecewise(double start) {
        return new PiecewiseInterpolator(start);
    }

    default Interpolator scale(double factor) {
        return input -> this.interpolate(input) * factor;
    }

    default Interpolator offset(double offset) {
        return input -> this.interpolate(input) + offset;
    }

    default Interpolator offsetInput(double offset) {
        return input -> this.interpolate(input + offset);
    }
    default Interpolator scaleInput(double scale) {
        return input -> this.interpolate(input * scale);
    }

    default Interpolator clamp(double min, double max) {
        return input -> {
            double value = this.interpolate(input);
            if (value < min) return min;
            if (value > max) return max;
            return value;
        };
    }


    default Interpolator andThen(Interpolator after) {
        return input -> after.interpolate(this.interpolate(input));
    }

    default Interpolator compose(Interpolator before) {
        return input -> this.interpolate(before.interpolate(input));
    }

}

