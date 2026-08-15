package com.rmsftc.library.control;

public class DoublePIDFController extends PIDFController<Double> {

    public DoublePIDFController(DoublePIDFCoeffs coefficients) {
        super(coefficients);
    }
    public DoublePIDFController() {
        super();
    }

}
