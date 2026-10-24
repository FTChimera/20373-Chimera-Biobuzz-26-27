package org.firstinspires.ftc.teamcode.util.control;

import java.util.Arrays;
import java.util.function.Function;

public class DoublePIDFCoeffs extends PIDFCoefficients<Double> {
    public static Function<Double, double[]> identityStateSupplier = d -> new double[]{d};
    public static Function<double[], Double> identityInverseStateSupplier = d -> Arrays.stream(d).sum();
    public DoublePIDFCoeffs(double P, double I, double D, double F) {
        super(identityStateSupplier, identityInverseStateSupplier);
        setPIDF(P, I, D, F);
    }
    public DoublePIDFCoeffs() {
        super(identityStateSupplier,identityInverseStateSupplier);
    }

    public com.pedropathing.controllers.PIDController getAsPedroPathing() {
        return new com.pedropathing.controllers.PIDController(Arrays.stream(p).sum(), Arrays.stream(i).sum(), Arrays.stream(d).sum());
    }
    public com.qualcomm.robotcore.hardware.PIDFCoefficients getAsFtcSdk() {
        return new com.qualcomm.robotcore.hardware.PIDFCoefficients(Arrays.stream(p).sum(), Arrays.stream(i).sum(), Arrays.stream(d).sum(), Arrays.stream(f).sum());
    }
}
