package org.firstinspires.ftc.teamcode.library;

public class Tuple<A,B> {
    A a;
    B b;
    public Tuple(A a, B b) {
        this.a = a;
        this.b = b;
    }

    public A getA() {
        return a;
    }
    public B getB() {
        return b;
    }
}
