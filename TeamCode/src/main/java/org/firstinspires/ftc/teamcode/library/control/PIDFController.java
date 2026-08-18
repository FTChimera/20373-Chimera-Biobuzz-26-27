package org.firstinspires.ftc.teamcode.library.control;

import java.util.function.Function;

public class PIDFController<T> {
    private PIDFCoefficients<T> coefficients;
    private T target;
    private T output;
    private double[] lastError;
    private double[] integral;
    private boolean firstUpdate = false;
    public double dt;
    private double lastTime;
    public void setStateSupplier(Function<T,double[]> stateSupplier) {
        this.coefficients.stateSupplier = stateSupplier;
    }
    public void setInverseStateSupplier(Function<double[],T> inverseStateSupplier) {
        this.coefficients.inverseStateSupplier = inverseStateSupplier;
    }
    public PIDFController(PIDFCoefficients<T> coefficients) {
        setCoefficients(coefficients);
    }
    public PIDFController() {
        setCoefficients(new PIDFCoefficients<>(null, null));
    }
    public void setCoefficients(PIDFCoefficients<T> coefficients) {
        this.coefficients = coefficients;
    }
    public void setTarget(T target) {
        this.target = target;
    }

    public T getTarget() {
        return target;
    }
    public T lastError() {
        return coefficients.inverseStateSupplier.apply(lastError);
    }
    public PIDFCoefficients<T> getCoefficients() {
        return coefficients;
    }
    public void reset() {
        firstUpdate = false;
        integral = null;
        lastError = null;
        dt = 0;
        lastTime = 0;
    }

    public boolean atTarget() {
        return atTarget(lastError);
    }
    public boolean atTarget(double[] error) {
        if (!firstUpdate) return false;
        for (int i = 0; i < error.length; i++) {
            if (Math.abs(error[i]) > coefficients.tolerance[i]) return false;
        }
        return true;
    }



        /** Updates the PIDF Controller. Should be called periodically.
         * @param state the current reading from sensor or elsewhere
         * */
    public void update(T state) {
        try {
            if (!firstUpdate) {
                lastError = new double[coefficients.stateSupplier.apply(state).length];
                integral = new double[coefficients.stateSupplier.apply(state).length];
                lastTime = System.nanoTime();
                firstUpdate = true;
            }
            dt = (System.nanoTime() - lastTime) / 1e9;
            lastTime = System.nanoTime();
            double[] currentStateArray = coefficients.stateSupplier.apply(state);
            double[] targetArray = coefficients.stateSupplier.apply(target);

            double[] error = new double[currentStateArray.length];
            for (int i = 0; i < currentStateArray.length; i++) {
                error[i] = targetArray[i] - currentStateArray[i];
            }
            coefficients.update(error);
            double[] outputArray = new double[error.length];
            for (int i = 0; i < error.length; i++) {
                integral[i] += error[i] * dt;
                // Anti-windup: clamp integral
                if (integral[i] > coefficients.integralLimit[i]) integral[i] = coefficients.integralLimit[i];
                if (integral[i] < -coefficients.integralLimit[i]) integral[i] = -coefficients.integralLimit[i];

                outputArray[i] = coefficients.p[i] * error[i];
                outputArray[i] += coefficients.d[i] * (error[i] - lastError[i]) / dt;
                outputArray[i] += coefficients.i[i] * integral[i];
                outputArray[i] += coefficients.f[i] * Math.signum(error[i]);
                if (atTarget(error)) outputArray[i] = 0;
            }
            this.output = coefficients.inverseStateSupplier.apply(outputArray);
            lastError = error;
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    public T getOutput() {
        return output;
    }

}
