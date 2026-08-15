package org.firstinspires.ftc.teamcode.pedroPathing.custom;

import com.pedropathing.geometry.CustomCurve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.pedropathing.math.Vector;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.PathPoint;

import com.rmsftc.library.math.Equation;

import java.util.Objects;

/**
 * EquationCurve - A custom curve defined by a expression equation
 * <p>
 * This class represents a curve defined by a expression equation, where the start and end poses are specified.
 * The expression must pass through the start and end poses to ensure continuity.
 * <p>
 * Key Features:
 * - Start and end poses that define the endpoints of the curve
 * - A expression that defines the shape of the curve
 * - Validation to ensure the expression passes through the specified poses
 *
 * @author Team
 */
public class EquationCurve extends CustomCurve {
    private static final double ENDPOINT_TOLERANCE = 1e-3;

    public final Pose startPose;
    public final Pose endPose;
    public final Equation equation;
    public final HeadingInterpolator interpolator;

    public EquationCurve(Pose startPose, Pose endPose, Equation equation, HeadingInterpolator interpolator) {
        super(() -> startPose, () -> endPose);
        this.startPose = Objects.requireNonNull(startPose, "startPose");
        this.endPose = Objects.requireNonNull(endPose, "endPose");
        this.equation = Objects.requireNonNull(equation, "equation");
        this.interpolator = Objects.requireNonNull(interpolator, "interpolator");
        validatePoses();
        if (!functionPassesThroughStartEnd()) {
            throw new IllegalArgumentException(
                    "The given equation does not pass through the start and end poses: " + equation);
        }
        initialize();
    }

    private boolean functionPassesThroughStartEnd() {
        boolean startMatches = MathFunctions.roughlyEquals(
                equation.getY(startPose.getX()), startPose.getY(), ENDPOINT_TOLERANCE);
        boolean endMatches = MathFunctions.roughlyEquals(
                equation.getY(endPose.getX()), endPose.getY(), ENDPOINT_TOLERANCE);
        return startMatches && endMatches;
    }

    private void validatePoses() {
        if (!Double.isFinite(startPose.getX()) || !Double.isFinite(startPose.getY())
                || !Double.isFinite(endPose.getX()) || !Double.isFinite(endPose.getY())) {
            throw new IllegalArgumentException("Equation curve poses must have finite x and y coordinates");
        }
        if (Double.compare(startPose.getX(), endPose.getX()) == 0) {
            throw new IllegalArgumentException(
                    "Equation curves require distinct start and end x coordinates");
        }
    }

    @Override
    public String pathType() {
        return "Equation Curve";
    }

    @Override
    public EquationCurve getReversed() {
        return new EquationCurve(endPose, startPose, equation, interpolator.reverse());
    }

    /**
     * This method returns the derivative of the curve at a given parameter t.
     *
     * @param t the parametric t value of the curve, which is to be clamped between 0 and 1 inclusive.
     * @return the derivative vector at the given parameter t.
     */
    @Override
    public Vector getDerivative(double t) {
        t = MathFunctions.clamp(t, 0, 1);

        double x = t * (endPose.getX() - startPose.getX()) + startPose.getX();
        double dxDt = endPose.getX() - startPose.getX();
        double dyDt = equation.derivative().getY(x) * dxDt;

        Vector derivative = new Vector();
        derivative.setOrthogonalComponents(dxDt, dyDt);
        return derivative;
    }

    /**
     * This method returns the pose of the curve at a given parameter t.
     *
     * @param t the parametric t value of the curve, which is to be clamped between 0 and 1 inclusive.
     * @return the pose at the given parameter t.
     */
    @Override
    public Pose getPose(double t) {
        t = MathFunctions.clamp(t, 0, 1);
        double x = t*(endPose.getX()-startPose.getX())+startPose.getX();
        Pose returned = new Pose(x, equation.getY(x));
        PathPoint closestPoint = new PathPoint(t, returned, null);
        return returned.setHeading(interpolator.interpolate(closestPoint));
    }

    /**
     * This method returns the second derivative of the curve at a given parameter t.
     *
     * @param t the parametric t value of the curve, which is to be clamped between 0 and 1 inclusive.
     * @return the second derivative vector at the given parameter t.
     */
    @Override
    public Vector getSecondDerivative(double t) {
        t = MathFunctions.clamp(t, 0, 1);

        double dxDt = endPose.getX() - startPose.getX();
        double x = t * dxDt + startPose.getX();
        double d2yDt2 = equation.derivative().derivative().getY(x) * dxDt * dxDt;

        Vector secondDerivative = new Vector();
        secondDerivative.setOrthogonalComponents(0, d2yDt2);
        return secondDerivative;
    }
}
