package org.firstinspires.ftc.teamcode.util.interpolation;

import java.util.ArrayList;
import java.util.List;

public class PiecewiseInterpolator implements Interpolator {
    public PiecewiseInterpolator(double start) {this.lastSegmentStart = start;}
    protected static class PiecewiseInput {
        double segmentStart;
        double segmentEnd;
        Interpolator interpolator;
        public PiecewiseInput(double segmentStart, double segmentEnd, Interpolator interpolator) {
            this.segmentStart = segmentStart;
            this.segmentEnd = segmentEnd;
            this.interpolator = interpolator;
        }
    }

    protected final List<PiecewiseInput> segments = new ArrayList<>();

    protected double lastSegmentStart;

    public PiecewiseInterpolator until(double end, Interpolator interpolator) {
        segments.add(new PiecewiseInput(lastSegmentStart, end, interpolator));
        lastSegmentStart = end;
        return this;
    }


    @Override
    public double interpolate(double input) {
        for (PiecewiseInput segment : segments) {
            if (input >= segment.segmentStart && input <= segment.segmentEnd) {
                return segment.interpolator.interpolate(input);
            }
        }
        return segments.get(segments.size() - 1).interpolator.interpolate(input); // extrapolation
    }
}
