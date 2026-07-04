package org.firstinspires.ftc.teamcode.pedroPathing.custom;

import com.pedropathing.geometry.CustomCurve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.pedropathing.math.Vector;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.PathPoint;

import org.firstinspires.ftc.teamcode.library.math.Equation;

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
    public Pose startPose;
    public Pose endPose;
    public Equation equation;
    public HeadingInterpolator interpolator;

    public EquationCurve(Pose startPose, Pose endPose, Equation equation, HeadingInterpolator interpolator) {
        super(() -> startPose, () -> endPose);
        if (!functionPassesThroughStartEnd(equation, startPose, endPose)) {
            throw new IllegalArgumentException("The given expression does not pass through start and end poses, " + equation);
        }
        this.startPose = startPose;
        this.endPose = endPose;
        this.equation = equation;
        this.interpolator = interpolator;
        initialize();
    }

    private boolean functionPassesThroughStartEnd(Equation equation, Pose startPose, Pose endPose) {
        boolean startMatches = MathFunctions.roughlyEquals(equation.getY(startPose.getX()), startPose.getY(), 1e-3);
        boolean endMatches = MathFunctions.roughlyEquals(equation.getY(endPose.getX()), endPose.getY(), 1e-3);
        return startMatches&&endMatches;
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
        // Will return zero for any expression whose 2nd derivative is just 0, so no safety checks needed
        double d2yDt2 = equation.derivative().derivative().getY(x) * dxDt * dxDt;

        Vector secondDerivative = new Vector();
        secondDerivative.setOrthogonalComponents(0, d2yDt2);
        return secondDerivative;
    }
}
