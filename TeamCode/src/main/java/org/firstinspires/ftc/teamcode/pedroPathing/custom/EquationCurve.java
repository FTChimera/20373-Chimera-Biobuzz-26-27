package org.firstinspires.ftc.teamcode.pedroPathing.custom;

import com.pedropathing.geometry.CustomCurve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.pedropathing.math.Vector;

import org.firstinspires.ftc.teamcode.library.math.Polynomial;

/**
 * EquationCurve - A custom curve defined by a polynomial equation
 * <p>
 * This class represents a curve defined by a polynomial equation, where the start and end poses are specified.
 * The polynomial must pass through the start and end poses to ensure continuity.
 * <p>
 * Key Features:
 * - Start and end poses that define the endpoints of the curve
 * - A polynomial that defines the shape of the curve
 * - Validation to ensure the polynomial passes through the specified poses
 *
 * @author Team
 */
public class EquationCurve extends CustomCurve {
    public Pose startPose;
    public Pose endPose;
    public Polynomial polynomial;

    public EquationCurve(Pose startPose, Pose endPose, Polynomial polynomial) {
        super(() -> startPose, () -> endPose);
        if (!polynomialPassesThroughStartEnd(polynomial, startPose, endPose)) {
            throw new IllegalArgumentException("Polynomial does not pass through start and end poses");
        }
        this.startPose = startPose;
        this.endPose = endPose;
        this.polynomial = polynomial;
        initialize();
    }

    private boolean polynomialPassesThroughStartEnd(Polynomial polynomial, Pose startPose, Pose endPose) {
        boolean startMatches = MathFunctions.roughlyEquals(polynomial.getY(startPose.getX()), startPose.getY(), 1e-3);
        boolean endMatches = MathFunctions.roughlyEquals(polynomial.getY(endPose.getX()), endPose.getY(), 1e-3);
        return startMatches&&endMatches;
    }

    @Override
    public String pathType() {
        return "Equation Curve";
    }

    @Override
    public EquationCurve getReversed() {
        return new EquationCurve(endPose, startPose, polynomial);
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
        double dyDt = polynomial.derivative().getY(x) * dxDt;

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
        return new Pose(x, polynomial.getY(x));
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
        double d2yDt2 = 0;
        if (polynomial.getDegree() >= 2) {
            double x = t * dxDt + startPose.getX();
            d2yDt2 = polynomial.derivative().derivative().getY(x) * dxDt * dxDt;
        }

        Vector secondDerivative = new Vector();
        secondDerivative.setOrthogonalComponents(0, d2yDt2);
        return secondDerivative;
    }
}
