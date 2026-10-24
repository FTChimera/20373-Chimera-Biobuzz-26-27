package org.firstinspires.ftc.teamcode.util.interpolation;

public class LUTInterp extends PiecewiseInterpolator {

    private final double min;
    private boolean hasSegments;
    private double lastEndValue;

    public LUTInterp(double min) {
        super(0);

        if (!Double.isFinite(min) || min <= 0) {
            throw new IllegalArgumentException(
                    "min must be finite and greater than zero"
            );
        }

        this.min = min;
    }

    @Override
    public LUTInterp until(double end, Interpolator interpolator) {
        if (interpolator == null) {
            throw new IllegalArgumentException(
                    "Interpolator cannot be null"
            );
        }

        validateEnd(end);

        // Extend the first segment backward to x = 0,
        // holding the value at x = min constant.
        if (!hasSegments) {
            double minValue = interpolator.interpolate(min);

            if (!Double.isFinite(minValue)) {
                throw new IllegalArgumentException(
                        "Interpolator must produce a finite value at min"
                );
            }

            super.until(
                    min,
                    Interpolator.constant(minValue)
            );
        }

        super.until(end, interpolator);

        lastEndValue = interpolator.interpolate(end);
        hasSegments = true;

        return this;
    }

    public LUTInterp until(double end, double endValue) {
        validateEnd(end);

        if (!Double.isFinite(endValue)) {
            throw new IllegalArgumentException(
                    "End value must be finite"
            );
        }

        double start = hasSegments ? lastSegmentStart : 0;
        double startValue = hasSegments ? lastEndValue : 0;

        return until(
                end,
                Interpolator.linear(
                        start, startValue,
                        end, endValue
                )
        );
    }

    private void validateEnd(double end) {
        if (!Double.isFinite(end)) {
            throw new IllegalArgumentException(
                    "End must be finite"
            );
        }

        if (end <= lastSegmentStart || end <= min && !hasSegments) {
            throw new IllegalArgumentException(
                    "End must be greater than the previous segment endpoint"
            );
        }
    }
}